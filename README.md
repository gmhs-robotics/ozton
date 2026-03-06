# ozton

Ozton is the Rust way of creating a playback/record auton for any vexide robot.

1. It should record and replay routes fast and iteratively.
2. It should be assisted by GPS/other sensors
3. The same frame system powers the drivetrain.

---

drivetrains could also be implemented in here too.but ill start with what i had
for push-back and lock in to make it it's own extensible library kinda like
evian + autons + my code

not this:

1. It should allow you to create autonomous routes by either designing them with
   the editor or recording them. Recordings may be played back raw or processed
   using the editor to use additional sensors and smoothing to ensure proper
   playback.
2. It should allow you to create the drive function that powers the autonomous
   routes easily.
3. Drivetrains and motion models should be constructable.

this is not for anybody, it is for ME. and ME alone. do NOT expand to other
people ever. i WANT:

- a way to record auton routes, that has proved to work fairly well!
- but that has 2 problems: mechanical drift + starting position
- so we use GPS to get position and velocity and heading, then we can do some
  fancy math to pathfind using a premade game field using coordinates. auton can
  then be manually programmed, but that's not what we want! its too hard.

What i was trying to do:

- support any drivetrain configuration :vomit:
- be evian without evian. I may still do this if i want but i dont need to

visitor pattern NO. we dont want: robot.grab(Object::Spawn2).visit( blah blah
blah ) we want to record and replay accurately
