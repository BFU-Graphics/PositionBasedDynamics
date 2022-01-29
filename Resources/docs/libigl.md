# LIBIGL API Reference

##  Viewer API

### Systematic Methods

```c++
int launch(bool resizable = true, bool fullscreen = false, const std::string &name = "libigl viewer", int width = 0, int height = 0);

```

### Viewport Methods

```c++
ViewerCore& core(unsigned core_id = 0);
int append_core(Eigen::Vector4f viewport, bool append_empty = false);
bool erase_core(const size_t index);
size_t core_index(const int id) const;
void select_hovered_core();
```

### Mesh Methods

```c++
bool load_mesh_from_file(const std::string & mesh_file_name);
bool   save_mesh_to_file(const std::string & mesh_file_name);

ViewerData& data(int mesh_id = -1);
int append_mesh(bool visible = true);
bool erase_mesh(const size_t index);
size_t mesh_index(const int id) const;
```

### Fields

```c++
// Data
std::vector<ViewerData> data_list;
size_t selected_data_index;
int next_data_id;

// Viewer
GLFWwindow* window;
std::vector<ViewerCore> core_list;
size_t selected_core_index;
int next_core_id;

// Plugin
std::vector<ViewerPlugin*> plugins;

// Callbacks
std::function<bool(Viewer& viewer)> callback_init;
std::function<bool(Viewer& viewer)> callback_pre_draw;
std::function<bool(Viewer& viewer)> callback_post_draw;
std::function<bool(Viewer& viewer, int button, int modifier)> callback_mouse_down;
std::function<bool(Viewer& viewer, int button, int modifier)> callback_mouse_up;
std::function<bool(Viewer& viewer, int mouse_x, int mouse_y)> callback_mouse_move;
std::function<bool(Viewer& viewer, float delta_y)> callback_mouse_scroll;
std::function<bool(Viewer& viewer, unsigned int key, int modifiers)> callback_key_pressed;
std::function<bool(Viewer& viewer, int w, int h)> callback_post_resize;
// THESE SHOULD BE DEPRECATED:
std::function<bool(Viewer& viewer, unsigned int key, int modifiers)> callback_key_down;
std::function<bool(Viewer& viewer, unsigned int key, int modifiers)> callback_key_up;
// Pointers to per-callback data
void* callback_init_data;
void* callback_pre_draw_data;
void* callback_post_draw_data;
void* callback_mouse_down_data;
void* callback_mouse_up_data;
void* callback_mouse_move_data;
void* callback_mouse_scroll_data;
void* callback_key_pressed_data;
void* callback_key_down_data;
void* callback_key_up_data;
```

## ViewerData API

```c++
// Empty all fields
void clear();

// Change the visualization mode, invalidating the cache if necessary
void set_face_based(bool newvalue);

// Helpers that can draw the most common meshes
void set_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
void set_vertices(const Eigen::MatrixXd& V);
void set_normals(const Eigen::MatrixXd& N);
void set_visible(bool value, unsigned int core_id = 1);
```

