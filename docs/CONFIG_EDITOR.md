# AgriRover Parameter Editor

The **AgriRover Parameter Editor** is a graphical tool designed to simplify the configuration of rover settings. It provides a user-friendly interface to edit the ROS 2 YAML configuration files used at startup.

## Features

- **Node-Based Grouping**: Parameters are automatically organized by their ROS 2 node namespace (e.g., `/rv1/navigator`, `/rv1/gps_driver`).
- **Contextual Tooltips**: Hover your mouse over any parameter name to see a detailed description of its function, units, and recommended values.
- **Type Safety**: Automatically detects and enforces data types (Integer, Float, Boolean, String) to prevent configuration errors.
- **File Management**: Load and edit different configuration files for multiple rovers or environments.

## How to Run

From the project root directory, execute:

```powershell
python tools/config_editor.py
```

## Interface Overview

1.  **File Selection**: The top bar shows the currently loaded file. Use the **Open...** button to load a different `.yaml` file.
2.  **Parameter List**: Scroll through the list of nodes. Each parameter has a label and an input field.
3.  **Tooltips**: Hover over a parameter name to see its description.
4.  **Save Changes**: Click the green **Save Changes** button to write your modifications back to the YAML file.
5.  **Reload**: Discard unsaved changes and reload the current file.

## Important Notes

- **Offline Editing**: This tool modifies the files on disk. Changes will **not** take effect on a running rover until the corresponding ROS 2 nodes are restarted.
- **Backup**: It is recommended to keep a copy of your original YAML files before making significant changes.
- **Invalid Input**: If you enter a value that doesn't match the expected type (e.g., text in a numeric field), the editor will show an error and prevent saving.
