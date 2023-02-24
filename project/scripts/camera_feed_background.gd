extends Sprite3D


@export var vertical_fov := true
@export var fov := 75.0


var _camera_feeds: Array[CameraFeed]
var _current_camera_feed: CameraFeed


func _ready() -> void:
    CameraServer.camera_feed_added.connect(self.update_cameras)
    CameraServer.camera_feed_removed.connect(self.update_cameras)
    update_cameras()


func update_cameras(_id := -1) -> void:
    _camera_feeds.clear()
    for i in CameraServer.get_feed_count():
        var feed = CameraServer.get_feed(i)
        _camera_feeds.push_back(feed)
    
    if _camera_feeds.size() == 0:
        _current_camera_feed = null
    else:
        if _current_camera_feed != _camera_feeds[0]:
            activate_feed(_camera_feeds[0])


func activate_feed(feed: CameraFeed) -> void:
    var tex := CameraTexture.new()
    tex.set_camera_feed_id(feed.get_id())
    feed.set_format(1, {})
    texture = tex
    _current_camera_feed = feed
    feed.frame_changed.connect(fit_screen, CONNECT_ONE_SHOT)
    feed.feed_is_active = true


func fit_screen() -> void:
    var size := sin(deg_to_rad(fov) / 2.0) * 2.5 * -position.z
    var pixels: float = texture.get_height() if vertical_fov else texture.get_width()
    pixel_size = size / pixels
