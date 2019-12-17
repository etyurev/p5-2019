# P5
# How to scan weld seam:
   - Open KUKA_KR60-3 project in the editor 
   - Place the workpiece in front of the robot
   - Jog robot manually to the welded seam in order to find the start ponint for the scan
   - Use data from the robot to specify startScanPose in the Main2Curves.py
   - Specify the velocity and angles for PIT in Main2Curves.py 
   - Run the program Main2Curves.py
   - In WorkVisual create the project to read robot's traces
   - Start tracing
   - While RoboDK launched, run ScanProgram from the GUI of RoboDK.
   - Transfer traces to the computer from KRC4 (.IO.r64 and .Ipo.r64 files for creating the PIT curves) to TransformClass
   - Transfer data from scanner to compute PIT positions


## How to compute PIT trajectory:
```
  - Scan weld seam.
  - Place weld_seam_scan.txt (name irrelevant) into 'scans/before/unprocessed/'
  - Edit program.py, line 30, change FILE_NAME = "weld_seam_scan"      .... Notice there is no file extension.
  - in terminal run: "python3 program.py"
  - Trajectory files are located in 'trajectories/' named weld_seam_scan-trajectory-0.txt and weld_seam_scan-trajectory-1.txt
  
```
# How to PIT the weld seam:
  - .txt files for left and right sides of the weld seam places inside TransformClass in KUKA_KR60-3 project
  - Run the program Main2Curves.py
  - Start tracing in WorkVisual
  - run CurveFollow1 and CurveFollow2 simultaneously in order to PIT the work piece
  - Transfer and save traces to computer to use it for velocity control and debuging
 


## How to compute quality report:
```
  - Scan treated weld seam with same trjectory used to compute PIT trajectory
  - Place weld_seam_scan-Quality.txt into 'scans/after/unprocessed
  - Edit quality_scan.py line 30 change FILE_NAME = "weld_seam_scan-Quality" ... Notice there is no file extension
  - in terminal run: "python3 quality_scan.py"
  - Output is graphs of quality parameters.
```
