(define
    (problem final-project)
    (:domain kitchen)

    (:objects
        stove - stovetop
        counter - countertop
        drawer - drawer
        cabinet - cabinet
        sugar spam - thing
        robot - gripper
    
    )

    (:init
        (closed drawer)
        (closed cabinet)
        (occupied stove)
        (occupied counter)
        (what-occupied stove sugar)
        (what-occupied counter spam)
        (gripper-loc robot counter)
;       (test robot)
    )

    (:goal (and
        (what-occupied counter sugar)
        (what-occupied drawer spam)
        (not(gripper-occupied robot))
    )
    )


)