extends Panel

@export var zoom_rate := 5.0
@export var camera_h_fov := 75.0

@onready var range_label: Label = $ZoomControl/Range
@onready var zoom_in: Button = $ZoomControl/ZoomIn
@onready var zoom_out: Button = $ZoomControl/ZoomOut
@onready var camera_3d: Camera3D = $SubViewportContainer/SubViewport/Camera3D


var view_range := 16.0 : set = set_view_range


func _ready() -> void:
    set_view_range(view_range)

func set_view_range(value: float) -> void:
    if value < 1.0:
        value = 1.0
    
    view_range = value
    range_label.text = "%3.1f m" % view_range
    camera_3d.size = view_range

func _draw() -> void:
    pass

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
    if zoom_in.button_pressed:
        set_view_range(view_range - zoom_rate * delta)
    if zoom_out.button_pressed:
        set_view_range(view_range + zoom_rate * delta)
