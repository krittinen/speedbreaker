# Play testing: what to send when something goes wrong

Thanks for testing. The game keeps its own logs and can pack everything we
need into one file, so a report takes one button press and a few words
from you.

## Installing

- **Steam Deck, Steam Machine, Steam Frame (SteamOS) and Mac:** the downloads
  on the Releases page, with the setup steps in the [README](../README.md)
  (and the README inside each SteamOS download). The first launch without
  game files offers to install them from your own disc image or extracted
  folder. In Game Mode, set the shortcut's Properties > General > Game
  Resolution to **Native**.
- **From source:** the README's "Building from source", or
  [scripts/steamos/README.md](../scripts/steamos/README.md) for SteamOS.
- **Windows (a beta build for testers):** unzip the kit somewhere (not
  inside the zip) and run `SpeedBreaker.exe`; its `README-TESTER.txt` has
  the rest. It is made with the `windows-clang-tester` CMake preset (no
  console window: everything goes to the session log; labelled "Windows
  beta") and packed with `scripts/windows/make_zip.py`, which checks that no
  game file, user name or user-folder path goes in.

## The menu

- Keyboard: **F1** (on a Mac also **Cmd+,**).
- Controller: **Back + Start** together (on a Steam Deck: the View and Menu
  buttons on either side of the screen).
- The D-pad picks a row, left/right changes it, A selects, LB/RB switch
  pages, B closes. The game pauses while the menu is open.
- The **Controls** page: choose a row, Left or Right for its first or second key, press A (or
  Enter), then the key you want; Esc or B cancels. Y (Backspace) clears a key, X (Delete) restores
  the row, *Reset page* the whole table.

## When something goes wrong

1. Open the menu, go to the **Advanced** page (LB/RB, or Q/E) and choose
   **Save Bug Report** (A, or Enter).
2. A notice at the bottom of the screen says where the file went:
   `speedbreaker-bug-report-v<version>-<date>_<time>.zip` (a Windows beta adds
   its label: `...-windows-beta-<build date>-<commit>.zip`) on your
   **Desktop** (on SteamOS, `/home/deck/Desktop`: switch to Desktop Mode to
   pick it up; on iPhone and iPad, the Files app's SpeedBreaker folder; on
   Windows, the Desktop Explorer shows, OneDrive's if it backs that up), or,
   if there is no Desktop (or it can't take the file), in the game's data
   folder (below).
3. Send that .zip with a few words: what you were doing, what you expected,
   what happened, and roughly how long into the session (the logs count
   seconds from launch).

With **Smooth Motion** on (Settings > Display, on a Mac's 120 Hz screen, or a 120 or 90 Hz one on
SteamOS or Linux), say so, and for a picture
problem whether it showed in a race, a cutscene or a menu: the log records each step between full
in-between pictures, lighter ones and 60 ("[hfr]" lines), and when.

If the game **crashed**, just launch it again: it says it saved a crash
report, and the next Save Bug Report includes it.

If the game **froze** (no picture change for 20 seconds, outside loading),
it writes a hang report by itself and says so. Open the menu and save a bug
report right there if you can; if you can't, quit (Steam's Exit Game, or
close the window), launch again and save one then: the hang report is kept.

A bug report holds the last five sessions' logs, any crash and hang
reports, your settings.toml, a summary of the machine (OS, CPU, memory, GPU
and driver, display, the settings in effect) and a screenshot of the last
game frame without the menu over it. Paths in your home folder show as `~`
(on Windows, your user folder shows as `%USERPROFILE%`). Nothing is sent
anywhere by the game: you send the file.

## Where the logs live

Each launch writes its own log (the newest five are kept), named after the
date and time it started, next to any `crash-*.txt` and `hang-*.txt`
reports:

- Linux / SteamOS: `~/.local/share/speedbreaker/logs/`
  (`$XDG_DATA_HOME/speedbreaker/logs/` if that is set; in portable mode,
  `data/logs/` in the game's folder, next to `portable.txt`)
- macOS: `~/Library/Application Support/SpeedBreaker/logs/`
- Windows: `%LOCALAPPDATA%\SpeedBreaker\logs\` (paste it into Explorer's
  address bar; in portable mode, `data\logs\` next to `SpeedBreaker.exe`
  and `portable.txt`). Keep `SpeedBreaker.pdb` next to `SpeedBreaker.exe`:
  crash and hang reports take the names of the game's functions from it.

On SteamOS the launcher also leaves the latest session's output in
`last-run.log`, in the game's folder (`~/Games/SpeedBreaker/` for the
download).

The first lines of every log say which build it is
(`[build] SpeedBreaker v<version> (<commit>), built ...`, or for a Windows
beta `[build] SpeedBreaker v<version> Windows beta <date> (<commit>): built
...`, also shown at the bottom of Settings > Advanced), whether this version
is new since the last run (`[version] updated from v<a> to v<b>`, or `first
run`), and describe the machine (`[system] ...`).

## Pixel shader diagnostics

To identify a draw's pixel shader, launch with `NFSMW_LOG_DRAWS=<seconds>`:
the log records one frame's draws, pixel-shader hashes and sampled texture
addresses. To skip every draw using a particular pixel shader, relaunch with
`NFSMW_SKIP_PS=<hex-hash>` (the hash after `ps_` in a `[draws]` line). This is
a diagnostic override; a shader shared by multiple effects will be skipped
in all of them.

The Motion Blur setting (Settings > Graphics, `NFSMW_MOTION_BLUR=off` for one
run) switches off the full-screen filter pass `ps_d52a5eb6c17f9598`'s blur: it
forces that shader's per-pixel blend weight to 0 (dropping the
neighbouring-tap contribution) while the rest of the draw, which also
composites the scene, still runs. The shaders it can alter are the
`kShaderOverrides` table in `runtime/gpu/renderer.cpp`, matched by the hash of
the shader's microcode; an entry logs a message and leaves the shader as is if
its expected GLSL statement isn't found. `NFSMW_LOG_PS_CONSTS=<hex-hash>`
logs that shader's used constants every 15th frame, with the fastest car's
speed (`NFSMW_LOG_C5=1` logs the command-processor writes carrying a nonzero
blur strength; `NFSMW_DUMP_GUEST=<hex-addr>[,<words>]` dumps guest memory
beside it; `NFSMW_TRACE_GUEST_ADDR=<hex-addr>[,<hex-bytes>]` logs the host
code that writes a guest range, on Windows).

F9 cycles the motion vector view (off; the vectors over the whole picture; split,
the picture on the left half and the vectors on the right, a black line between,
to watch the blur and the vectors together): the same pass draws the camera's motion
vectors in place of the picture (Middlebury colour wheel: hue is the
direction, saturation the length, full at 24 pixels; white is still). The
vectors come from the depth buffer (the pass's `t2d_4`, read as z/w),
reprojected with the previous picture's view-projection
(`runtime/gpu/motion_vectors.h`; the matrices are read in the view setup hook
in `game/hfr.cpp`, the player's view id 1). A camera cut shows no motion.
Switching it on compiles shader variants (one hitch). Vehicles show their own
motion, exactly over their silhouette: while the view is on, every draw into
the scene's targets (colour base 0, pitch 1280) also attaches a host-only
RGBA16F velocity target as colour 3 (cleared each picture, so a pass does not
split), and a vehicle's own opaque draws write it through a pixel shader
variant: the fragment's NDC (from `gl_FragCoord`) and depth are reprojected
through the vehicle's previous matrix (world-before * view-projection-before,
matched across pictures by render object), as the camera's are through the
previous view-projection. A vehicle's draw is recognised by its vertex shader's
c0-c3 holding the vehicle's world * view-projection as columns, and must write
depth (the translucent shadow blob under a car uses the same matrices). The
blur pass reads the target (binding 31) over the camera's vector. Objects not
yet placed (a zero position) are skipped. Rigid motion only: wheels' rotation
is not seen.

Motion Blur: Per-object (Settings > Graphics, `NFSMW_MOTION_BLUR=per_object`;
experimental) uses the same vectors without F9: the same blur pass
(`ps_d52a5eb6c17f9598`) has its own blend of eight fixed taps replaced by a
blur sampled along each pixel's motion (the camera's from the depth; with
`NFSMW_MV_VEHICLES=0` (or F10) turns off the vehicle's, which a vehicle's draw wrote, on
by default (off, the scenery's blur can be judged alone). The strongest motion of a nearer
object within 14 pixels takes over a pixel's (so a fast object's blur covers
the edge of its silhouette), samples nearer than the pixel are not blurred into
it, the length is capped, and the sample positions are jittered per pixel.
`NFSMW_MV_BLUR_SHUTTER=<0.05-2>` (default 0.6, the part of a picture's motion
the blur spans), `NFSMW_MV_BLUR_MAX=<2-200>` (default 40, the longest blur in
pixels of the 1280x720 picture) and `NFSMW_MV_BLUR_KNEE=<0-60>` (default 6: an
object's own motion under that many pixels is not blurred, ramping in up to 2.3
times as many; the camera's motion has no such floor) set it. The game's own radial blur is not
applied in this mode. `NFSMW_DUMP_GLSL=<hex hash>` writes a pixel shader's
translated GLSL to `ps_<hash>.glsl`.

Motion vector diagnostics (all log to stderr, `NFSMW_LOG_MV=1`):
`NFSMW_LOG_OFFSETS=1` (`[mvl]` the vehicle list, `[mvy]` how many draws of a car
hold its matrix exactly, `[mvu]` draws near a car that its path did not take),
`NFSMW_LOG_JUMPS=1` (`[mvj]` parts whose movement changed visibly from the
picture before), `NFSMW_LOG_LIGHTS=1` (`[mvz]` draws at a car that write no
depth). `NFSMW_MV_NO_LOCAL=1` turns the per-part vehicle matching off,
`NFSMW_MV_NO_MARKER=1` the "not written" marker draws, `NFSMW_MV_NO_SMOOTH=1` (and
`NFSMW_MV_SMOOTH_PICTURES=<1-6>`, default 6) turns off the smoothing of the camera's step over the last few
pictures (the camera's picture-to-picture step varies by about 50% at the 90th percentile; the average
of six varies by 11%), `NFSMW_MV_STATIC_WRITE=1` writes the camera's
vector for static draws, and `NFSMW_MV_WHEEL_SPIN=1` the experimental wheel
rotation.
F11 logs a marker (`[mark] #n frame F guest pictures P`) with a short toast, to
note the moment something looked wrong; the frame is the renderer's, the same as
the `[mv...]` lines'. F10 toggles the vehicles' own motion vectors (a toast says whether they are enabled
or disabled; they are on by default, `NFSMW_MV_VEHICLES=0` starts with them off). Off, every pixel, the
cars' too, has the camera's motion from the depth. Only the game's listed cars are
followed, and only draws of the vertex shaders `f3cb08fa`, `31a7fc62` and `50eb5376`
can give a vehicle's vector (`NFSMW_MV_VEHICLE_VS=<hex>[,<hex>...]` adds shaders);
the matching of a draw to a car by anything else (`NFSMW_MV_GENERIC=1`) took trees
and props for moving things. `NFSMW_LOG_CAR_OBJECT=1` logs each vehicle object's floats
once (`[carobj]`) to find its fields.

`NFSMW_LOG_CAR_DRAWS=1` logs, once per vertex shader and constant slot, where
a vehicle's world matrix, its world-view-projection, the view-projection or a
vehicle's position sits in the vertex shader constants (`[cardraw]` lines; the
vehicles' matrices come from the render list read at the view setup, per
picture). It is the first step of per-vehicle motion vectors.
