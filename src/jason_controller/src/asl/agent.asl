
/* Initial beliefs and rules */

//at(X).
//
///* Initial goals */
//!check(rooms).
//
///* Plans */
//+!visit(Location)
//    <- at(Location).
//
//+!check(rooms) : door(Room,open)
//    <- next(room);
//        !check(rooms).
//+!check(rooms).
//+door(Door,closed) <-.print("Found closed door!").



/* Initial beliefs and rules */
at(X).

/* Initial goals */
!check(rooms).

/* Plans */
+!visit(Location)
    <- at(Location).

+!check(rooms) : room_checks_finished(false)
    <- !check(doors);
        next(Room);
        !check(rooms).
+!check(rooms).


+!check(doors) : door_checks_finished(false)
    <-  next(Door);
        !check(doors).
+!check(doors).

+room_checks_finished(true) <-.print("All rooms checked!").
+door_checks_finished(true) <-.print("All doors in room checked!").
