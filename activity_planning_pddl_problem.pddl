(define
    (problem final-project)
    (:domain kitchen)

    (:objects
        l-stove r-stove - stovetop
        l-counter c-counter r-counter - countertop
        r-drawer g-drawer b-drawer - drawer
        l-cabinet c-cabinet r-cabinet - cabinet
        sugar spam - thing
        gripper - gripper
    
    )

    (:init
        (closed r-drawer)
        (closed g-drawer)
        (closed b-drawer)
        (closed l-cabinet)
        (closed c-cabinet)
        (closed r-cabinet)
        (occupied l-stove)
        (occupied c-counter)
        (what-occupied l-stove sugar)
        (what-occupied c-counter spam)
        (gripper-loc c-counter)
    )

    (:goal (and
        (what-occupied ?c-counter ?sugar)
        (what-occupied ?r-drawer ?spam)
        (not(gripper-occipied ?gripper))
    )
    )


)