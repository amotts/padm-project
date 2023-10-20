(define
    (domain kitchen)
    (:requirements :strips :typing :negative-preconditions)
    (:types
        gripper location thing - object
        sugar spam - thing
        stovetop countertop cabinet drawer - location
    )

    (:predicates
        (occupied ?l - location)
        (what-occupied ?l - location ?t - thing)
        (closed ?l - location)
        (gripper-loc ?g - gripper ?l - location)
        (item-held ?g - gripper ?t - thing)
        (gripper-occupied ?g - gripper)
    )

    (:action OPEN_DOOR
        :parameters (?g - gripper ?l - location)
        :precondition (and 
            (gripper-loc ?g ?l)
            (closed ?l)
            (not(gripper-occupied ?g))
        )
        :effect (not (closed ?l))
    )

    (:action CLOSE_DOOR
        :parameters (?g - gripper ?l - location)
        :precondition (and 
            (gripper-loc ?g ?l)
            (not(closed ?l))
            (not(gripper-occupied ?g))
        )
        :effect (closed ?l)
    )

    (:action GRAB_THING
        :parameters (?g - gripper ?l - location ?t - thing)
        :preconditions(and
            (not(closed ?l))
            (what-occupied ?l ?t)
            (gripper-loc ?g ?l )
            (not (gripper-occupied ?g))
        )
        :effect(and
        (gripper-occupied ?g)
        (item-held ?g ?t)
        )
    )

    (:action RELEASE_THING
        :parameters (?g - gripper ?l - location ?t - thing)
        :preconditions(and
            (what-occupied ?l ?t)
            (gripper-loc ?g ?l )
            (item-held ?g ?t)
        )
        :effect(and
        (not (gripper-occupied ?g))
        (not (item held ?g ?t))

        )
    )

    (:action MOVE_OBJECT
        :parameters (?g - gripper ?l1 - location ?l2 - location ?t - thing)
        :precondition (and
            (not (closed ?l1))
            (not (closed ?l2))
            (not (occupied ?l2))
            (what-occupied ?l1 ?t)
            (gripper-loc ?g ?l1)
            (item held ?g ?t)
        )
        :effect (and
            (not (what-occupied ?l1 ?t))
            (not (occupied ?l1))
            (occupied ?l2)
            (what-occupied ?l2 ?t)
            (not(gripper-loc ?g ?l1))
            (gripper-loc ?g ?l2)
        )
    )
    
    (:action MOVE_ARM
        :parameters (?g - gripper ?l1 - location ?l2 - location)
        :precondition (and
            (gripper-loc ?g ?l1)
            (not(gripper-occupied ?g))
        )
        :effect (and
            (not(gripper-loc ?g ?l1))
            (gripper-loc ?g ?l2)
        )
    )

)