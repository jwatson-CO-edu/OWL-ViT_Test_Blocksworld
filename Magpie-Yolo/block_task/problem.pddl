(define
	(problem blocks)
	(:domain blocks)
	(:objects
		redBlock yellowBlock blueBlock loc-a loc-b loc-c - object
	)
	(:init (Block redBlock) (Block blueBlock) (Block yellowBlock) (on yellowBlock blueBlock) (on blueBlock loc-a) (on redBlock loc-c) (fixed loc-a) (fixed loc-b) (fixed loc-c) (clear yellowBlock) (clear redBlock) (clear loc-b))
	(:goal (on blueBlock yellowBlock))
)
