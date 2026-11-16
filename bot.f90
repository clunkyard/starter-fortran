! A robot on the Clunkyard core's Fortran binding. The core owns tracking,
! geometry and protocol handling; these procedures choose the tactics.
! It searches with a broad scan, narrows the beam on a contact, and leads
! shots from a line through its recent sightings.
! Clunkyard robot API 2.0.0: one core under every language, the same
! answers everywhere. Facts (a call shows its arguments):
! match: mode ruleset control robots is_3d tick_rate max_ticks
!   robot_id(slot)
! clock: tick seconds remaining_ticks
! arena: size gravity terrain_kind ground_height(x y)
! me: health position velocity heading pitch roll angular_velocity
!   attitude grounded altitude applied hull_mounts to_mount(heading pitch)
!   from_mount(heading pitch) mount_toward(point)
! battery: charge capacity regen sustainable_drive compute
!   drive_cost(fraction)
! drive: hover_lift
! gun: aim_now cooldown_ticks spread max_power damage_scale
!   can_fire(power reserve) shot_speed(power) shot_cost(power)
!   reload_ticks(power) shot_damage(power) power_for_speed(speed)
!   power_for_flight_time(range seconds) lead(contact power)
!   lead_point(position velocity power)
! scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
! sensors: contacts nearest cues contact(slot) position_of(contact)
! tracker: cue nearest_bearing_rate velocity(slot) position(slot)
!   history(slot) last_seen(slot)
! nav: zone zone.margin(point) zone.contains(point margin)
!   zone.distance_from(point) storm storm.margin(point)
!   storm.contains(point margin) storm.distance_from(point) hill
!   hill.margin(point) hill.contains(point margin)
!   hill.distance_from(point) pickups nearest_pickup(kind)
!   field_distance(a b)
! score: me best armour
! angles: normalize(degrees) angle_diff(target current)
!   bearing_to(from to) elevation_to(from to) direction(heading pitch)
!   distance(a b) to_chassis(heading pitch attitude)
!   from_chassis(heading pitch attitude) clamp(value low high)
! raw: number(path) text(path) has(path) flag(path) count(path) line()
! Controls: drive.set(throttle turn pitch_turn)
!   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
!   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
!   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
!   status.set(text value) debug.log(line) raw.extra(json)
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
integer :: dodge_ticks=0,dodges=0
integer(c_int32_t) :: slot=0
! Bearing is earned: a scan contact's bearing is off by up to its bearing_error
! (degrees, a fixed share of the arc), so a wide beam finds a robot and a narrow
! one aims at it. Contacts within 120 units are exact.
real(c_double), parameter :: search_arc=40.0_c_double  ! degrees: sweeps the circle in half a second
real(c_double), parameter :: hold_width=60.0_c_double  ! units across at the target: it stays in the beam from tick to tick
real(c_double), parameter :: min_arc=6.0_c_double      ! degrees: the narrowest the beam holds
integer(c_int32_t), parameter :: window=12             ! ticks of sightings the aim averages
integer, parameter :: sightings=8                      ! narrow sightings in the window before a shot
integer(c_int32_t), parameter :: look_every=40         ! ticks between looks around the target, for a nearer robot
integer(c_int32_t), parameter :: reacquire=10          ! ticks to look again where a lost target was
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
! A straight line through the narrowest of the last `window` ticks' sightings:
! where the robot is now and how fast it moves. Each sighting is off by its
! bearing_error; the line averages that out.
subroutine fit(handle,target_slot,tick,found,position,velocity,kept)
  type(c_ptr), value :: handle
  integer(c_int32_t), value :: target_slot,tick
  logical :: found
  type(cy_vec3) :: position,velocity
  integer :: kept
  type(cy_sighting) :: history(32)
  integer(c_int32_t) :: total,rate,code
  logical :: keep(32)
  real(c_double), allocatable :: t(:),x(:),y(:),z(:)
  real(c_double) :: best,t0,spread,sx,sy,sz
  found=.false.;kept=0
  if(cy_tracker_history(handle,target_slot,history,32_c_int32_t,total)/=CY_OK)return
  if(total<2)return
  keep(1:total)=history(1:total)%tick>tick-window
  if(count(keep(1:total))<2)return
  best=minval(history(1:total)%bearing_error,keep(1:total))
  ! Keep the narrow ones: a look's wide sighting drops out.
  keep(1:total)=keep(1:total).and.history(1:total)%bearing_error<=1.5_c_double*best
  kept=count(keep(1:total))
  if(kept<2)return
  t=real(pack(history(1:total)%tick,keep(1:total)),c_double)
  x=pack(history(1:total)%position%x,keep(1:total));y=pack(history(1:total)%position%y,keep(1:total));z=pack(history(1:total)%position%z,keep(1:total))
  t0=sum(t)/kept;t=t-t0;spread=sum(t*t)
  ! Least squares on each axis: slope per tick, and the line's position now.
  sx=sum(t*(x-sum(x)/kept))/spread;sy=sum(t*(y-sum(y)/kept))/spread;sz=sum(t*(z-sum(z)/kept))/spread
  code=cy_match_tick_rate(handle,rate)
  position=cy_vec3(sum(x)/kept+sx*(tick-t0),sum(y)/kept+sy*(tick-t0),sum(z)/kept+sz*(tick-t0))
  velocity=cy_vec3(sx*rate,sy*rate,sz*rate);found=.true.
end subroutine
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
  integer(c_int32_t) :: code,tick,is_3d,can_fire,mode
  real(c_double) :: heading,pitch,throttle,power,delta,bearing,distance,elevation,error,across,up,arc,sweep,margin,away,control,climb,turning,pitching,work
  logical :: free_flight,fired,left,seen,found,aimed,ready
  integer(c_int32_t) :: last_seen
  integer :: fit_count
  type(cy_vec3) :: fit_position,fit_velocity,spot
  real(c_double) :: hold
  type(cy_contact) :: cues(32)
  integer(c_int32_t) :: heard
  code=cy_me_position(handle,position);code=cy_arena_gravity(handle,gravity);code=cy_scanner_last(handle,last)
  heading=get(handle,cy_me_heading);pitch=get(handle,cy_me_pitch)
  code=cy_clock_tick(handle,tick);code=cy_match_is_3d(handle,is_3d);free_flight=is_3d/=0.and.gravity%z==0
  code=cy_drive_brake(handle,0.2_c_double)
  seen=.false.;last_seen=0
  if(slot/=0)seen=cy_tracker_last_seen(handle,slot,last_seen)==CY_OK
  if(cy_sensors_nearest(handle,target)==CY_OK)then
    slot=target%slot
    throttle=0.2_c_double;if(target%range>350)throttle=0.7_c_double;if(target%range<200)throttle=-0.4_c_double
    call drive(handle,throttle,steer(target%bearing,heading),steer(target%elevation,pitch))
    ! Halve the beam each tick until it is hold_width across at the target:
    ! narrow enough for a precise bearing, wide enough to keep it in.
    hold=max(min_arc,2*atan2(hold_width/2,target%range)*180/acos(-1.0_c_double))
    ! In a crowd a narrow beam sees one robot. Every look_every ticks, widen it for a tick.
    if(mod(tick,look_every)==0)then
      arc=search_arc
    else
      arc=max(hold,min(search_arc,last%arc/2))
    end if
    call scan(handle,target%bearing,arc,target%elevation)
    code=cy_gun_power_for_flight_time(handle,target%range,1.0_c_double,power)
    if(target%bearing_error>0)then
      ! A noisy contact: lead the fitted line, and fire only once it rests on enough sightings.
      call fit(handle,target%slot,tick,found,fit_position,fit_velocity,fit_count)
      aimed=.false.;if(found)aimed=cy_gun_lead_point(handle,fit_position,fit_velocity,power,aim)==CY_OK
      ready=aimed.and.fit_count>=sightings
    else
      ! An exact contact: cy_gun_lead is the intercept if the target holds course.
      aimed=cy_gun_lead(handle,target,power,aim)==CY_OK;ready=.true.
    end if
    if(.not.aimed)then
      aim%heading=target%bearing;aim%pitch=target%elevation
    end if
    code=cy_gun_aim_heading(handle,aim%heading);code=cy_gun_aim_pitch(handle,aim%pitch)
    code=cy_gun_aim_now(handle,now);code=cy_angles_angle_diff(aim%heading,now%heading,delta);code=cy_gun_can_fire(handle,power,10.0_c_double,can_fire)
    if(ready.and.abs(delta)<3.and.abs(aim%pitch-now%pitch)<3.and.can_fire/=0)then
      code=cy_gun_fire(handle);code=cy_gun_power(handle,power)
    end if
    if(ready)then
      code=cy_status_text(handle,'TRACK',5_c_int32_t)
    else
      code=cy_status_text(handle,'NARROW',6_c_int32_t)
    end if
    code=cy_status_value(handle,target%range)
  else if(seen.and.tick-last_seen<=reacquire)then
    ! Lost it: look again where it was last seen, widening the beam, before searching.
    code=cy_tracker_position(handle,slot,spot);code=cy_angles_bearing_to(position,spot,bearing)
    elevation=0;if(free_flight)code=cy_angles_elevation_to(position,spot,elevation)
    call drive(handle,0.5_c_double,steer(bearing,heading),steer(elevation,pitch));call scan(handle,bearing,min(search_arc,last%arc*2),elevation)
    code=cy_status_text(handle,'REACQUIRE',9_c_int32_t)
  else if(cy_tracker_cue(handle,cue)==CY_OK)then
    code=cy_angles_bearing_to(position,cue%position,bearing);code=cy_angles_distance(position,cue%position,distance)
    elevation=0;if(free_flight)code=cy_angles_elevation_to(position,cue%position,elevation)
    error=min(cue%bearing_error,30.0_c_double);across=real(mod(tick,5)-2,c_double)/2*error;up=0
    if(free_flight)up=real(mod(tick/5,3)-1,c_double)*error/2
    call drive(handle,0.7_c_double,steer(bearing,heading),steer(elevation,pitch));code=cy_scanner_arc_for_reach(handle,distance+100,arc)
    call scan(handle,bearing+across,arc,elevation+up);code=cy_status_text(handle,'CUE',3_c_int32_t);code=cy_status_value(handle,distance)
  else
    code=cy_angles_normalize(last%heading+36,sweep);up=0;if(free_flight)up=real(mod(tick/10,3)-1,c_double)*35
    call drive(handle,0.5_c_double,0.33_c_double,steer(0.0_c_double,pitch));call scan(handle,sweep,search_arc,up)
    code=cy_status_text(handle,'SEARCH',6_c_int32_t)
  end if
  ! A simple dodge: vary it, or predict the shooter.
  fired=.false.
  if(cy_sensors_cues(handle,cues,size(cues),heard)==CY_OK)fired=any(cues(1:heard)%source==CY_SOURCE_MUZZLE)
  if(dodge_ticks==0.and.fired)then
    dodges=dodges+1;dodge_ticks=10
  end if
  if(dodge_ticks>0)then
    ! Odd dodges go left, even ones right; turn +1 and strafe -1 go left.
    left=mod(dodges,2)==1;work=merge(1.0_c_double,-1.0_c_double,left);dodge_ticks=dodge_ticks-1
    code=cy_match_control(handle,mode)
    if(mode==CY_CONTROL_FORCES)then
      code=cy_drive_throttle(handle,0.5_c_double);code=cy_drive_strafe(handle,-work)
    else
      code=cy_drive_throttle(handle,1.0_c_double);code=cy_drive_turn(handle,work)
    end if
    if(left)then
      code=cy_status_text(handle,'dodge:left',10_c_int32_t)
    else
      code=cy_status_text(handle,'dodge:right',11_c_int32_t)
    end if
    code=cy_status_value(handle,real(dodges,c_double))
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
end subroutine
end program
