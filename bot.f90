! A robot on the Clunkyard core's Fortran binding. The core owns tracking,
! geometry and protocol handling; these procedures choose the tactics.
! Clunkyard robot API 2.0.0: one core under every language, the same
! answers everywhere. Facts (a call shows its arguments):
! match: mode ruleset control robots is_3d tick_rate max_ticks
!   robot_id(slot)
! clock: tick seconds remaining_ticks
! arena: size gravity terrain_kind ground_height(x y)
! me: health position velocity heading pitch roll angular_velocity
!   attitude grounded altitude applied
! battery: charge capacity regen sustainable_drive compute
!   drive_cost(fraction)
! drive: hover_lift
! gun: aim_now cooldown_ticks spread max_power damage_scale
!   can_fire(power reserve) shot_speed(power) shot_cost(power)
!   reload_ticks(power) shot_damage(power) power_for_speed(speed)
!   power_for_flight_time(range seconds) lead(contact power)
!   lead_point(position velocity power)
! scanner: last proximity_range reach(arc) arc_for_reach(range)
! sensors: contacts nearest cues contact(slot) position_of(contact)
! tracker: cue nearest_bearing_rate velocity(slot) position(slot)
!   last_seen(slot)
! nav: zone zone.margin(point) zone.contains(point margin)
!   zone.distance_from(point) storm storm.margin(point)
!   storm.contains(point margin) storm.distance_from(point) hill
!   hill.margin(point) hill.contains(point margin)
!   hill.distance_from(point) pickups nearest_pickup(kind)
!   field_distance(a b)
! score: me best armour
! angles: normalize(degrees) angle_diff(target current)
!   bearing_to(from to) elevation_to(from to) direction(heading pitch)
!   distance(a b) clamp(value low high)
! raw: number(path) text(path) has(path) flag(path) count(path) line()
! Controls: drive.set(throttle turn pitch_turn)
!   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
!   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
!   tracker.configure(smoothing forget_ticks cue_ticks)
!   status.set(text value) raw.extra(json)
! Every control argument is optional; a second call keeps what the first
! set. Absent facts are your language's none. Names are snake_case here;
! your language cases them its own way. Angles are degrees from the +x
! axis; pitch is positive up.
! Manual: https://clunkyard.com/manual#sdk
program bot
use clunkyard
implicit none
type(c_ptr) :: robot=c_null_ptr
integer(c_int32_t) :: status,event
abstract interface
  function query(handle,out) bind(C) result(status)
    import
    type(c_ptr), value :: handle
    real(c_double) :: out
    integer(c_int32_t) :: status
  end function
end interface
status=cy_open(2_c_int32_t,0_c_int32_t,'clunkyard-fortran',17_c_int32_t,'2.0.0',5_c_int32_t,robot)
if(status<0)stop 1
do
  status=cy_session_next(robot,event)
  if(status==CY_E_INTERNAL)then
    status=cy_session_send(robot)
    if(status<0)exit
    cycle
  end if
  if(status/=CY_OK.or.event==CY_EVENT_END)exit
  if(event==CY_EVENT_STATE)call turn(robot)
  status=cy_session_send(robot)
  if(status<0)exit
end do
call cy_close(robot)
contains
function get(handle,read) result(value)
  type(c_ptr), value :: handle
  procedure(query) :: read
  real(c_double) :: value
  integer(c_int32_t) :: code
  value=0;code=read(handle,value)
end function
function steer(towards,current) result(value)
  real(c_double), value :: towards,current
  real(c_double) :: value,delta
  integer(c_int32_t) :: code
  code=cy_angles_angle_diff(towards,current,delta)
  code=cy_angles_clamp(delta/45,-1.0_c_double,1.0_c_double,value)
end function
subroutine drive(handle,throttle,turning,pitching)
  type(c_ptr), value :: handle
  real(c_double), value :: throttle,turning,pitching
  integer(c_int32_t) :: code
  code=cy_drive_throttle(handle,throttle);code=cy_drive_turn(handle,turning);code=cy_drive_pitch_turn(handle,pitching)
end subroutine
subroutine scan(handle,heading,arc,pitch)
  type(c_ptr), value :: handle
  real(c_double), value :: heading,arc,pitch
  integer(c_int32_t) :: code
  code=cy_scanner_heading(handle,heading);code=cy_scanner_arc(handle,arc);code=cy_scanner_pitch(handle,pitch)
end subroutine
! The arena's calibration kernel, xorshift-sqrt v1; safe to delete.
function calibration_work(work,tick) result(value)
  integer(c_int64_t), value :: work,tick
  integer(c_int64_t) :: x,i
  real(c_double) :: sum,value
  x=ieor(88172645463325252_c_int64_t,tick);sum=0
  do i=1,work
    x=ieor(x,shiftl(x,13));x=ieor(x,shiftr(x,7));x=ieor(x,shiftl(x,17))
    sum=sum+sqrt(real(shiftr(x,11),c_double))
  end do
  value=real(mod(int(floor(sum,kind=c_int64_t),c_int64_t)+shiftr(x,40),1000000_c_int64_t),c_double)
end function
subroutine turn(handle)
  type(c_ptr), value :: handle
  type(cy_vec3) :: position,gravity
  type(cy_contact) :: target
  type(cy_cue) :: cue
  type(cy_scan) :: last
  type(cy_solution) :: aim
  type(cy_direction) :: now
  type(cy_storm) :: storm
  type(cy_zone) :: zone
  integer(c_int32_t) :: code,tick,is_3d,can_fire
  real(c_double) :: heading,pitch,throttle,power,delta,bearing,distance,elevation,error,across,up,arc,sweep,margin,away,control,climb,turning,pitching,work
  logical :: free_flight
  code=cy_me_position(handle,position);code=cy_arena_gravity(handle,gravity);code=cy_scanner_last(handle,last)
  heading=get(handle,cy_me_heading);pitch=get(handle,cy_me_pitch)
  code=cy_clock_tick(handle,tick);code=cy_match_is_3d(handle,is_3d);free_flight=is_3d/=0.and.gravity%z==0
  code=cy_drive_brake(handle,0.2_c_double)
  if(cy_sensors_nearest(handle,target)==CY_OK)then
    throttle=0.2;if(target%range>350)throttle=0.7;if(target%range<200)throttle=-0.4
    call drive(handle,throttle,steer(target%bearing,heading),steer(target%elevation,pitch));call scan(handle,target%bearing,20.0_c_double,target%elevation)
    code=cy_gun_power_for_flight_time(handle,target%range,1.0_c_double,power)
    if(cy_gun_lead(handle,target,power,aim)/=CY_OK)then
      aim%heading=target%bearing;aim%pitch=target%elevation
    end if
    code=cy_gun_aim_heading(handle,aim%heading);code=cy_gun_aim_pitch(handle,aim%pitch)
    code=cy_gun_aim_now(handle,now);code=cy_angles_angle_diff(aim%heading,now%heading,delta);code=cy_gun_can_fire(handle,power,10.0_c_double,can_fire)
    if(abs(delta)<3.and.abs(aim%pitch-now%pitch)<3.and.can_fire/=0)then
      code=cy_gun_fire(handle);code=cy_gun_power(handle,power)
    end if
    code=cy_status_text(handle,'TRACK',5_c_int32_t);code=cy_status_value(handle,target%range)
  else if(cy_tracker_cue(handle,cue)==CY_OK)then
    code=cy_angles_bearing_to(position,cue%position,bearing);code=cy_angles_distance(position,cue%position,distance)
    elevation=0;if(free_flight)code=cy_angles_elevation_to(position,cue%position,elevation)
    error=min(cue%bearing_error,30.0_c_double);across=real(mod(tick,5)-2,c_double)/2*error;up=0
    if(free_flight)up=real(mod(tick/5,3)-1,c_double)*error/2
    call drive(handle,0.7_c_double,steer(bearing,heading),steer(elevation,pitch));code=cy_scanner_arc_for_reach(handle,distance+100,arc)
    call scan(handle,bearing+across,arc,elevation+up);code=cy_status_text(handle,'CUE',3_c_int32_t);code=cy_status_value(handle,distance)
  else
    code=cy_angles_normalize(last%heading+36,sweep);up=0;if(free_flight)up=real(mod(tick/10,3)-1,c_double)*35
    call drive(handle,0.5_c_double,0.33_c_double,steer(0.0_c_double,pitch));call scan(handle,sweep,40.0_c_double,up)
    code=cy_status_text(handle,'SEARCH',6_c_int32_t)
  end if
  if(get(handle,cy_battery_charge)<get(handle,cy_battery_capacity)/4)code=cy_drive_throttle(handle,get(handle,cy_drive_get_throttle)*0.5_c_double)
  if(cy_nav_storm(handle,storm)==CY_OK)then
    if(cy_nav_storm_margin(handle,position,margin)==CY_OK)then
      if(margin>-60)then
        code=cy_angles_bearing_to(storm%center,position,away);code=cy_angles_angle_diff(away,heading,delta)
        code=cy_angles_clamp(delta/30,-1.0_c_double,1.0_c_double,control);code=cy_drive_throttle(handle,0.8_c_double);code=cy_drive_turn(handle,control)
      end if
    end if
  end if
  if(cy_nav_zone(handle,zone)==CY_OK)then
    if(cy_nav_zone_margin(handle,position,margin)==CY_OK)then
      if(margin<60)then
        code=cy_angles_bearing_to(position,zone%center,bearing);climb=0
        if(zone%shape==CY_SHAPE_SPHERE)code=cy_angles_elevation_to(position,zone%center,climb)
        code=cy_angles_angle_diff(bearing,heading,delta);code=cy_angles_clamp(delta/30,-1.0_c_double,1.0_c_double,turning)
        code=cy_angles_clamp((climb-pitch)/30,-1.0_c_double,1.0_c_double,pitching);call drive(handle,0.8_c_double,turning,pitching)
      end if
    end if
  end if
  ! Only the arena's rate calibration sends this; safe to delete.
  if(cy_raw_number(handle,'state.calibration.work',22_c_int32_t,work)==CY_OK)then
    if(work>=1)then
      code=cy_status_text(handle,'WORK',4_c_int32_t);code=cy_status_value(handle,calibration_work(int(work,c_int64_t),int(tick,c_int64_t)))
    end if
  end if
end subroutine
end program
