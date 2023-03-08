extends Control


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
    visibility_changed.connect(func():
        set_physics_process(visible)
    )
    set_physics_process(visible)


func _draw() -> void:
    var center = size / 2
    var s = min(center.x, center.y) * 0.9
    draw_set_transform(center)
    var vx = Input.get_axis("backward", "forward")
    var vy = Input.get_axis("right", "left")
    draw_line(Vector2(0, 0), Vector2(-vy * s, -vx * s), Color.RED)
    draw_rect(Rect2(Vector2(-s, -s), 2 * Vector2(s, s)), Color.GRAY, false)
    var ang = Input.get_axis("turn_right", "turn_left")
    draw_line(Vector2(0, s), Vector2(-ang * s, s), Color.GREEN)


func _process(_delta: float) -> void:
    queue_redraw()
