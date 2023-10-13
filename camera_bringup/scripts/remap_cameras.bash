#!/bin/bash
sudo unlink /dev/camFront
sudo unlink /dev/camBack
sudo unlink /dev/camLeft
sudo unlink /dev/camRight

v4l2-ctl --list-devices > cam_devices.txt
SUB='Arducam_16MP: Arducam_16MP'
KERNEL_FRONT="2.1):"
KERNEL_BACK="2.2):"
KERNEL_LEFT="2.3):"
KERNEL_RIGHT="2.4):"
FOUND_FRONT=false
FOUND_BACK=false
FOUND_LEFT=false
FOUND_RIGHT=false
while read p; do
  FULL_LINE=$p
  echo $FULL_LINE
  if [ "$FOUND_FRONT" = true ]; then
      sudo ln -s "$p" "/dev/camFront"
      echo "sudo ln -s $p dev/camFront"
      FOUND_FRONT=false
  elif [ "$FOUND_BACK" = true ]; then
      sudo ln -s "$p" "/dev/camBack"
      echo "sudo ln -s $p dev/camBack"
      FOUND_BACK=false
  elif [ "$FOUND_LEFT" = true ]; then
      sudo ln -s "$p" "/dev/camLeft"
      echo "sudo ln -s $p dev/camLeft"
      FOUND_LEFT=false
  elif [ "$FOUND_RIGHT" = true ]; then
      sudo ln -s "$p" "/dev/camRight"
      echo "sudo ln -s $p dev/camRight"
      FOUND_RIGHT=false
  fi
  if [[ "$FULL_LINE" =~ .*"$SUB".* ]]; then
    port_number=${FULL_LINE:45}
    echo "$port_number"
    if [ "$port_number" = "$KERNEL_FRONT" ]; then
      FOUND_FRONT=true
    elif [ "$port_number" = "$KERNEL_BACK" ]; then
      FOUND_BACK=true
    elif [ "$port_number" = "$KERNEL_LEFT" ]; then
      FOUND_LEFT=true
    elif [ "$port_number" = "$KERNEL_RIGHT" ]; then
      FOUND_RIGHT=true
    fi
  fi
done <cam_devices.txt