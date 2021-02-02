
/* Initial beliefs and rules */

/* Initial goals */

!check(rooms).

/* Plans */

//Recursively checks each room
+!check(rooms) : not done(rooms)
    <-  !check(doors);
        !check(furniture);
        next(room);
        !check(rooms).
+!check(rooms).

//Recursively checks each door in a given room
+!check(doors) : not done(doors)
    <-  next(door);
        !check(doors).
+!check(doors).

//recursively checks every piece of furniture in a given room
+!check(furniture) : not done(furniture)
    <- next(furniture);
       !check(furniture).
+!check(furniture).

//recursively finds every object
+!check(objects) : not done(objects)
    <- next(object);
       !check(objects).
+!check(objects).

+target(Location,room)
    <-  moveTo(Location).

+target(Location,door)
    <-  moveTo(Location);
        inspect(Location).

+target(Location,furniture)
    <-  moveTo(Location);
        inspect(Location).

 +target(Location,object)
    <-  moveTo(Location);
        inspect(Location).
       
+!visit(Location)
    <- moveTo(Location).
    
+moved(Item)
    <- find(Item).

+closed
    <- open.

+done(rooms)
    <-  .drop_all_desires;
        .print("All rooms checked!");
        !check(objects);
        .print("Completed 'Getting to know my home'!");
        saveChanges.
        
+done(doors) <-.print("All doors in room checked!").
+done(furniture) <-.print("All furniture in room checked!").
+done(objects) <-.print("All moved objects have been found!").

+doorbellSounded
    <-  !visit(entrance);
        open.