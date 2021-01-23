
/* Initial beliefs and rules */

/* Initial goals */

!check(rooms).

/* Plans */
+!check(rooms) : not done(rooms)
    <- !check(doors);
        next(room);
        !check(rooms).
+!check(rooms).

+!check(doors) : not done(doors)
    <-  next(door);
        !check(doors).
+!check(doors).

+!check(furniture) : not done(furniture)
    <- next(furniture);
       !check(furniture).
+!check(furniture).
                
+target(Location,room)
    <-  move_to(Location).

+target(Location,door)
    <-  move_to(Location);
        inspect(Location).
        

+!visit(Location)
    <- move_to(Location).

+room_checks_finished <-.print("All rooms checked!").
+door_checks_finished <-.print("All doors in room checked!").
