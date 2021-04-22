
/* Initial beliefs and rules */

/* Initial goals */

//!check(rooms).
//!waitForBell.
//!waitForEntranceOpened.
//!getCommand.
!testGoal.

/* Plans */

+!testGoal
    <-  findObject(apple).



+!getCommand
    <- getCommand.
+!waitForBell
    <- waitForBell.

+!waitForEntranceOpened
    <- waitForEntranceOpened.

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
    <-  .print("Doorbell sounded!");
        !visit(entrance);
        open;
        !identifyVisitor.

+!identifyVisitor
    <-  scanFace.

+personIs(unknown)
    <-  interrogate.

+personIs(Person)
    <-  !welcome(Person).

+!welcome(dr_kimble)
    <-  escort(bedroom);
        waitUntilVisitorDone;
        escort(entrance);
        waitUntilVisitorLeft;
        closeDoor.
    
+!welcome(postman)
    <-  acceptMail;
        waitUntilVisitorLeft;
        closeDoor;
        !visit(bedroom);
        deliverMail.

+!welcome(deliman)
    <-  escort(kitchen);
        askToLeaveBreakfast;
        escort(entrance);
        waitUntilVisitorLeft;
        closeDoor.

+!welcome(plumber)
    <-  askPlumberDesiredRoom.

+plumberDesiredRoomIs(Room)
    <-  escort(Room);
        waitUntilVisitorDone;
        escort(entrance);
        waitUntilVisitorLeft;
        closeDoor.

+visitorOutOfBounds
    <- complain.


+grannyAlarm
    <-  !visit(dude);
        getCommand;
        executeCommand.


//recursively carry out individual elements of command
+!executeCommand : not done(commands)
    <- next(command);
       !executeCommand.
+!executeCommand
    <- .print("hgfhfg!").


+task(search)
    <- .print("search task!");
        chooseMostLikelyLocation;
        !visit(MostLikelyLocation).

//Recursively checks each location
+!check(likelyLocation) : not done(likelyLocations)
    <-  next(likelyLocation);
        !check(likelyLocations).
+!check(likelyLocations).


+task(accompany)
    <- .print("accompany task!").

+task(manipulate)
    <- .print("manipulate task!").


+entranceOpened
    <-  moveTo(wayPoint1);
        moveTo(wayPoint2);
        moveTo(wayPoint3);
        follow.


+obstacleDetected
    <-  identifyObstacle;
        moveObstacle.
