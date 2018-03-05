## Training Instructions/Guide ##

Directories
    data- holds the cascade model, parameter, and individual stage files
    neg- holds randomized photos of images not containing drones 
    pos- holds images of drones
    test- holds images and videos to test with

Files
    bg.txt- holds location data for each negative image
    drone.txt- holds location data as well as data to help pinpoint location of drone on image
    drone.vec- output vector file containing positive samples for training


### Getting Started ###
Instructions gathered from [OpenCV](https://docs.opencv.org/3.3.1/dc/d88/tutorial_traincascade.html)
1) Images must be gathered

2) Create bg.txt
    a) In directory: ls $PWD/*.jpeg $PWD/*.jpg $PWD/*.png > bg.txt
                   mv bg.txt ..

3) Create annotation file
    a) opencv_annotation --annotations=/path/to/annotations/file.txt --images=/path/to/image/folder/

4) Create vector output file
    a) opencv_createsamples \
        -info drone.txt \
        -bg bg.txt \
        -vec drone.vec \
        -num 2419 \
        -w 40 -h 40

5) Train Cascade Classifier
    a) opencv_traincascade \ 
    -data data \
    -vec drone.vec \
    -bg bg.txt \
    -numPos 2200 -numNeg 1100 -numStages 16 \
    -precalcValBufSize 1024 -precalcIdxBufSize 1024 \
    -featureType HAAR \
    -minHitRate 0.995 -maxFalseAlarmRate 0.5 \
    -w 40 -h 40
