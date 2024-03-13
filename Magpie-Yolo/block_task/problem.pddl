(define
	(problem blocks)
	(:domain blocks)
	(:objects
		redBlock yellowBlock blueBlock loc-a loc-b loc-c - object
	)
	(:init (Block redBlock) (Block blueBlock) (Block yellowBlock) (on blueBlock loc-a) (on yellowBlock loc-c) (on redBlock loc-b) (fixed loc-a) (fixed loc-b) (fixed loc-c) (clear blueBlock) (clear yellowBlock) (clear redBlock))
	(:goal (on blueBlock yellowBlock))
)
