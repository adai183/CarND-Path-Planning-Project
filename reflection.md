I took the approach presented in the walkthrough video as a starting point. Most of the trajectory generation is already taken care of by making extensive use of the spline library to fit a function to waypoints provided by the simulator. The lane change can be easily controlled by
i) the use of previous waypoints to avoid to extreme steering angles. This way we always get a smooth curve when transitioning from one lane to another.
ii) the use of Frenet Coordinates.  This way it is made easy to get the exact target position in a neighboring lane. We simply add the lane width.

As I said before being provided with this solution makes it easy to generate trajectories that keep our car centered in the lane and aldo trajectories to smoothly change lanes without violating the constraints on Max Acceleration and Jerk.

The one thing that still has to be taken care of is to make sure that our agent behaves well with respect to the other cars in the simulated environment.

I made up a really simple solution to fulfill the basic requirements. So here is my model:

First I check the potential target lanes the car can transition to in case we  want to perform a lane change. There are basically two case. The first case is where the current lane is the middle lane. In this case we have to potential target lanes: The outer right lane and the outer left lane. The other case is when the car is driving in one of outer lanes. For this case out target lane will be the middle lane because our agent will not perform double lane changes. Too risky for now.
Here is the code for this:

```c++
// set potential lane changes
vector<int> target_lanes;

if (lane == 1)
{
  target_lanes.push_back(0);
  target_lanes.push_back(2);
}
else
{
  target_lanes.push_back(1);
}
```

Next I initialize two flags.
`too_close` will be set to true in case we are getting too close to a car ahead of us in the same lane. `
Lane_change_safe` will be set to true in case it is safe for the agent to initialize a lane change toward a target lane.

Allright! Now we can finally take a look at the provided sensor fusion data to see what the other cars are doing.
First the frenet `d coordinate` is extracted from each car to check for cars in the same lane as the agent. We basically take a closer look at all the cars in the range width of the lane the car is currently driving.
In the second step we have to see if one of the detected cars is too close to the agent. Therefore we first have to project the detected cars position into the future in case we are working with previous waypoints. After that we set the `too_close` flag to true in case the distance is smaller than the hardcoded safety distance.
Here is the code for this:

```c++
// check if car in current lane is too close
for (size_t i = 0; i < sensor_fusion.size(); i++)
{
  // car is in my lane
  float d = sensor_fusion[i][6];
  if (d < (2+4*lane+2) && d > (2+4*lane-2) )
  {
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx + vy*vy);
    double check_car_s = sensor_fusion[i][5];

    check_car_s += ((double)prev_size*.02*check_speed); // when using previous points we need tp project values out

    // check s values greater than mine and s gap
    if ((check_car_s > car_s) && ((check_car_s - car_s) < SAFETY_DISTANCE) )
    {
      too_close = true;
    }

  }
}

```

Once we have detected that our agent is too close to another car ahead of him we have to decide what to do. There are two options: a) slowing down to avoid a crash b) change lane.
In order to see if option b) is a safe plan we have to see if one of the target lanes is safe to transition, too. We take a similar approach as before. First a find all the cars in the target lane with help of our frenet `d coord` and than whether there is enough space for the car in the longitudinal dimension. The difference is that this time we have to check for cars ahead us and also for cars behind us.


If this check confirms that the lane change is safe we can change lanes, right?? Not quite yet!
Imagine a case where there are slow cars ahead of us in both our current lane and the target lane. In this case the agent will abandon its plan to change lane before executing the full lane change. The result is an agent that is trapped between two lanes,  rapidly shifting between the two without ever reaching the center of either lane.  This is why I introduced a vote system. Every time a lane change passes the safety check we increase the cote score for the target lane. When a certain threshold is reached the lane change is finally carried out. This little trick smoothes out the decision process and makes sure our agent doesn’t drive like crazy in between two lanes.

Here is the code for this:

```c++

// check if there are other cars on the potential target lanes
for (size_t j = 0; j < target_lanes.size(); j++)
{

  for (size_t i = 0; i < sensor_fusion.size(); i++)
  {
    // car is in my lane
    float d = sensor_fusion[i][6];
    if (d < (2+4*target_lanes[j]+2) && d > (2+4*target_lanes[j]-2) )
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += ((double)prev_size*.02*check_speed); // when using previous points we need tp project values out

      // check s values greater than mine and s gap
      if ( ((check_car_s > car_s) && ((check_car_s - car_s) < SAFETY_DISTANCE)) || ((check_car_s < car_s) && ((car_s - check_car_s) < SAFETY_DISTANCE)) )
      {
        lane_status[target_lanes[j]] = 1;
      }
    }
  }
}
```

Now I only have to take car the lane change is carried out in case the threshold is reached and reset the lane score to 0. In case no lane change is performed we make sure the agent slows down to avoid a crash.
Here is the code for this:

```c++
if (!lane_change_safe) {
    ref_vel -= .224;
  }

}
else if (ref_vel < 49.5) { //49.5
  ref_vel += .224;
}
```
