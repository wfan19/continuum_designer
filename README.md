# CONTINUUM DESIGNER

A set of tools for analyzing the design of an antagonistic and tapered continuum arm using the general statics model for continuum arms under tip loading as desrcibed in Olson 2020.

### Getting Started:
To download the project using git:

```
git clone --recurse-submodules git@github.com:wfan19/continuum_designer.git
```

Alternatively, you can also download the files directly as a zip file. However, the submodules `include/continuum_solver` and `include/Adjoint` will not be populated, so you will have to download them as well and place the files into the corresponding directories. Those projects can be found [here](https://github.com/wfan19/Adjoint/tree/main) and [here](https://github.com/wfan19/continuum-solver/tree/2023_rewrite). However, do note that this may cause compatibility issues due to dependency version mismatches.

Then, within Matlab, add the "include" directory and its subdirectories into the Matlab include path.

Finally, run `antagonist_arm_sandbox.m` - you should now see an antagonistic arm with 4 actuators that you can play around with in simulation.
