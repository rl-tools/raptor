ffmpeg -y -i '/Volumes/DataPuddle3/web app.mov' \
  -vf "scale=350:-1:flags=lanczos,format=rgb24,palettegen=max_colors=48" \
  -frames:v 1 palette16.png

ffmpeg -i '/Volumes/DataPuddle3/web app.mov' -i palette16.png \
  -lavfi "fps=50,scale=350:-1:flags=lanczos[x];[x][1:v]paletteuse=dither=none" \
  -loop 0 raptor.rl.tools.gif
