Tools:<br />
C++<br />
VS Command Prompt<br /> 
FFmpeg<br />
<br />
Execute Commands For Video Generation:<br />
cl /EHsc videoGeneration.cpp<br />
videoGeneration.exe<br />
ffmpeg -r 30 -i frame_%d.ppm -vcodec libx264 -pix_fmt yuv420p output.mp4<br />
<br />
Execute Command for Distance:<br />
python depthDistance.py<br />
(Update CSV File to Current One)<br />
<br />
depthWait: Does depth camera readings at intervals based on 100 LiDAR readings<br />
<br />
depthRead: Reads depth camera every 3 pixels without LiDAR<br />
<br />
finalDistance: Attempts to average and modify buzzing pulse using computed averages<br />
<br />
distanceTest: Finds average if depth camera reading is available and takes LiDAR reading if not<br />
<br />
