echo "Configuring and building ORB_SLAM2 ..."

rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j