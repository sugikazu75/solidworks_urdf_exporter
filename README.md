# SolidWorks to URDF Exporter

Authored and maintained by [Stephen Brawner](brawner@gmail.com). Past supporters include [PickNik Consulting](https://picknik.ai), Verb Surgical, Open Robotics, and Willow Garage. 

## Latest Release

**SolidWorks 2021**

https://github.com/ros/solidworks_urdf_exporter/releases/tag/1.6.1

**SolidWorks 2020**

https://github.com/ros/solidworks_urdf_exporter/releases/tag/1.6.0

**SolidWorks 2019 on 2018 SP 5**

https://github.com/ros/solidworks_urdf_exporter/releases/tag/1.5.1

## SolidWorks Version Requirements

1. The minimum required version of SolidWorks for use with this add-in is 2018 Service Pack 5. SolidWorks 2017 or earlier may work. See [this issue](https://github.com/ros/solidworks_urdf_exporter/issues/73).

## Usage

See the [ROS Wiki](http://wiki.ros.org/sw_urdf_exporter) and associated [tutorials](http://wiki.ros.org/sw_urdf_exporter/Tutorials).

### Reference-only links

In the assembly exporter, leave **Link Components** empty to export a reference
frame without inertial, visual, collision, or mesh data. For a non-base link,
select an existing reference coordinate system for its origin; it is connected
to its parent by a fixed joint, so no reference axis is needed. Empty links may
have children. An empty base link may also use the automatically generated global
coordinate system.

### Overridden mass properties

With SolidWorks 2020 or later, assembly links use component-level mass properties,
including values assigned through **Override Mass Properties**, in SI units.
This also preserves component placement when a link contains a subassembly.
Older SolidWorks versions, or documents for which `CreateMassProperty2` is
unavailable, use the previous body-based calculation and log a warning. That
fallback does not support overridden mass properties and retains the previous
subassembly limitations. The separate part exporter is unchanged.

Building this version requires the .NET Framework 4.5.2 targeting pack and
SolidWorks 2020 or newer interop assemblies.

The assembly export screen has separate **Mesh Format** (STL / 3dxml / DAE) and
**Robot Format** (URDF / Xacro) choices. URDF remains the default. Selecting
Xacro writes `urdf/<package-name>.xacro` with
`xmlns:xacro="http://www.ros.org/wiki/xacro"` on the `robot` element. It preserves
the existing link and joint definitions; it does not generate macros or parameters.
The generated ROS launch files process the file with Xacro before loading it,
and `package.xml` includes the `xacro` runtime dependency. Both export buttons
(with and without meshes) use the selected robot format.

### DAE meshes (color)

Select **DAE (color)** in the assembly export screen to write COLLADA 1.4.1
meshes directly, without an external converter. SolidWorks component overrides,
face, feature, body and part RGB colors are written as opaque diffuse materials.
Images, decals, textures, transparency and reflective appearances are not exported.
The URDF/Xacro omits its single visual material for DAE so viewers can use the
embedded mesh colors. Link material controls therefore do not recolor DAE meshes.

Geometry is expressed in meters in the link coordinate system, including nested
component placement; inertial properties are unchanged. The existing relative
mesh-folder setting and export-without-meshes option also work with DAE.
DAE uses the model's current display tessellation, rather than the STL coarse/fine
setting. Adjust SolidWorks image quality and rebuild to change its resolution.
Components must be resolved and have tessellation available; export fails with
an error instead of silently generating an empty mesh when triangles are missing.
This option applies to assembly export; the separate part exporter is unchanged.

### STL face limit

In the assembly export screen, select **STL**, then **Max faces** under Mesh Detail.
The numeric field defaults to **199999** and accepts values from 4 to 199999.
These settings belong to the selected link and are saved with the exporter configuration.
Older configurations retain the previous Coarse/Fine behavior. To limit every STL,
select Max faces for every physical link before export.

This mode starts from a fine STL and simplifies it only if it exceeds the specified
number of triangular faces (not vertices). It uses geometry3Sharp's quadric-error
edge-collapse simplifier, the same family of algorithms as MeshLab's Quadric Edge
Collapse Decimation. Small details can change; coordinates, units, and CAD-derived
mass/inertia are retained. Degenerate faces are cleaned and the final binary STL
face count is checked before replacing the full-resolution output. If valid
connectivity cannot be reconstructed or the limit cannot be reached, export fails
and the full-resolution STL is retained for diagnosis. Coarse/Fine, DAE, 3dxml,
export-without-meshes, and the separate part exporter do not apply this limit.

Restore NuGet packages when building: the signed `geometry3Sharp-signed` 1.0.323
dependency is copied alongside the add-in DLL and included by the existing installer.
No MeshLab installation or .NET Framework upgrade is required.

## Development

1. Install Visual Studio 2017
1. Install .NET desktop development
    1. From Visual Studio: `Tools > Get Tools and Features...`
    1. Check `.NET desktop development` package
    1. Select `Modify`
1. Install the [SolidWorks API tools](https://help.solidworks.com/2019/english/api/sldworksapiprogguide/GettingStarted/SolidWorks_API_Getting_Started_Overview.htm)
1. Launch Visual Studio with admin privileges. Right click and select `Run as Administrator`
1. Open `sw2urdf/SW2URDF.sln`  
1. Enable Debugging
    1. Right click `SW2URDF` in the Solution Explorer
    1. Click the `Debug` Tab
    1. Ensure `Configuration:` is set to `Debug`
    1. Ensure `Start external program:` is pointing to the SolidWorks executable. For example `C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\SLDWORKS.exe`

## Converting mesh format from 3dxml to dae

Executing the following command will convert the format of the exported mesh from 3DXML to DAE, and rewrite the URDF, allowing you to display colored meshes in visualization tools like RViz:

```bash
pip3 install scikit-robot -U
convert-urdf-mesh <URDF_PATH> --output <OUTPUT_URDF_PATH>
```

### Trouble Shooting

1. `AxImp.exe` error - Check the installation of the .Net Tools. If there is no error, install the Windows 10 SDK.
1. `Resourse.resx` error - Check if `sw2urdf/SW2URDF/Resources.resx` exists and is empty. If empty, delete this file then right click the `SW2URDF` in the Solution Explorer and select `Properties`. Navigate to the Resources tab and click the button to create a new file.
