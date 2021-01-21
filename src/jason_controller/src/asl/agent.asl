/* Initial beliefs and rules */
at(outside).

/* Initial goals */
!start.
!visit(inside).

/* Plans */
+!start : true <- test_ros_communication.
+!visit(Location)
    <- at(Location).
