cd ..
axel -an 8 https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz -o rgbd_freiburg1_desk.tgz
tar -xf rgbd*.tgz
cd rgbd_dataset*
python3 associate.py rgb.txt depth.txt > associate.txt
