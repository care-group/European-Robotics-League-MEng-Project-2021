
/* Initial beliefs and rules */

/* Initial goals */

!check(rooms).

/* Plans */

//Recursively checks each room
+!check(rooms) : not done(rooms)
    <- !check(doors);
        next(room);
        !check(rooms).
+!check(rooms).

//Recursively checks each door in a given room
+!check(doors) : not done(doors)
    <-  next(door);
        !check(doors).
+!check(doors).

//ecursively checks every piece of furniture in a given room
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

+closed
    <- open.

+done(rooms) <-.print("All rooms checked!");
                .drop_all_desires;
                saveChanges.
+done(doors) <-.print("All doors in room checked!").
