#!/bin/bash
DEST_DIR=/home/pi/ws/picam
SHM_DIR=/run/shm

mkdir -p $SHM_DIR/rec
mkdir -p $SHM_DIR/hooks
mkdir -p $SHM_DIR/state
mkdir -p $SHM_DIR/hls

mkdir -p $DEST_DIR/archive

ln -sfn $DEST_DIR/archive $SHM_DIR/rec/archive
ln -sfn $SHM_DIR/rec $DEST_DIR/rec
ln -sfn $SHM_DIR/hooks $DEST_DIR/hooks
ln -sfn $SHM_DIR/state $DEST_DIR/state

./picam --alsadev hw:1,0 -o /run/shm/hls/ -w 1440 -h 1088 --time --timeformat "%Y-%m-%d %H:%M:%S" -v 1536000 -g 60

#-o /run/shm/hls/
