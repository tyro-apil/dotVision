#!/bin/bash

user=$(whoami)

BASE_MEDIA_DIR="/media/${user}"

DEST_DIR="/home/${user}/pdfs"

copy_pdfs() {
  while true; do
      for MEDIA_DIR in "$BASE_MEDIA_DIR"/*; do
        if [ -d "$MEDIA_DIR" ] && mountpoint -q "$MEDIA_DIR"; then
          cp "$MEDIA_DIR"/*.pdf "$DEST_DIR"
        fi
      done
      sleep 10
  done
}

copy_pdfs &
