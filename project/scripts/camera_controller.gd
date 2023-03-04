extends Camera3D


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
    pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta: float) -> void:
    var b = Basis.IDENTITY \
        .rotated(Vector3.FORWARD, OpenTrack.rpy.x) \
        .rotated(Vector3.LEFT, OpenTrack.rpy.y) \
        .rotated(Vector3.UP, OpenTrack.rpy.z)
    basis = b
