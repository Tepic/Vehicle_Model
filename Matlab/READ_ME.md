# TESTS

**DoubleLaneChange** - it is designed to simulate control for the standard ISO3888 Double Lane Change ( **DLC** ) test. Downside of this simulation test is that the speed is maintained during the simulated test. It is needed to implement friction model in order to give additional action-reaction to have an influence on the vehicle speed and handling.

**userDefinedTest** - it present a code for different vehicle handlings - *FREE TEST*


**v07** - its is the **main** where everything originates 

# FUNCTIONS
Additional functions used in two previous tests:

*doubleLaneChange* - it generates cones positions for DLC and gates in positions which are input parameters

*plotData* - it plots all simulation parameters. The last input argument must be: {'DLC', 'etc'}. *DLC* is for Double-Lane-Change test, whilst *etc* is for everything else for now (free test)

*generateOrientation* - it is used for generating control reference for car handling. Reference is car orientation over time. It returns a vector of vehicle's orientation over time and it also returns time vector. The last input argument is one of {'ramp', 'step'}. It defines the time of generated control reference - does the control reference changes with unit step type or it is more "smooth" using ramp type.

# Additional files

*tire_Avon* - it holds the data about tires used on FSRA16 (Road Arrow vehicle from 2016)

*wheel_speeds* - angular speed of each wheel generated from previous simulations (currently unused so far, since the control of vehicle is not implemented to control wheels, only steering wheel). For such improvements, it is needed to add momentum equations about load and speed of each wheel over the whole system (vehicle). System is already too non-linear, therefor this was not implemented due to deadline to finish final Bachelor thesis. This is one of the following steps of improvements.
