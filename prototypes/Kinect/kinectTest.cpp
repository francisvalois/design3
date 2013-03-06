//
//  main.cpp
//  test
//
//  Created by Francis Valois on 2013-02-27.
//  Copyright (c) 2013 Francis Valois. All rights reserved.
//

#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"
#include "Utility.h"
#include "kinect.h"

using namespace cv;
using namespace std;

namespace {

    TEST(KinectTest, GetDistanceForObstacle){
        //Arrange
        Mat testMatrix = Utility::readFromFile("matrix3.yml");
        Kinect kinect;
        
        //Act
        kinect.findCenteredObstacle(testMatrix);
        Vec2f obstacle1 = kinect.getObstacle1();
        float positionX1 = obstacle1[0];
        float positionZ1 = obstacle1[1];
        Vec2f obstacle2 = kinect.getObstacle2();
        float positionX2 = obstacle2[0];
        float positionZ2 = obstacle2[1];
        
        //Assert
        //Accept a range of distances for X:0.805 and Z:2.2209
        ASSERT_TRUE(positionX1 >= 0.805-0.01 && positionX1 < 0.805 + 0.01);
        ASSERT_TRUE(positionZ1 >= 2.2209-0.01 && positionZ1 < 2.2209 + 0.01);
        
        //Accept a range of distances for X:0.258 and Z:1.337
        ASSERT_TRUE(positionX2 >= 0.258-0.01 && positionX2 < 0.258 + 0.01);
        ASSERT_TRUE(positionZ2 >= 1.337-0.01 && positionZ2 < 1.337 + 0.01);       
        
    }
    
    TEST(KinectTEST, getTrueCoordFromKinectCoords){
        //Arrange
        Kinect kinect;
        Vec3f depthXYZ(1,2,3);
        
        //Act
        Vec2f trueCoords = kinect.getTrueCoordFromKinectCoord(depthXYZ);
        
        //Assert a range
        ASSERT_TRUE(trueCoords[0] >= (0.42 - 0.3) || trueCoords[0] <= (0.42 + 0.3));
        ASSERT_TRUE(trueCoords[1] >= (2.59 - 0.3) || trueCoords[1] <= (2.59 + 0.3));
    }

}

int main(int argc, char * argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

