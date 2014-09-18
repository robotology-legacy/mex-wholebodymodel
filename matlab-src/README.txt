More detailed documentation and comments throughout the code will be provided soon.

To run the simulator, run the 'run_forwardDyn.m' file. User can change various parameters of the simulation by modifying 'set_parameters.m' file which can also be accessed from the GUI. GUI gives the options to set simulation duraion and step times which by default are set to 10 and 0.01 seconds relatively. Also the checkbox 'Plots' allows the plotting of the contact forces at the feet, joint torques and center of mass trajectory errors.

To test different configurations, go to "set_parameters.m"; change EXPERIMENT_MODE to 1; change VISUALIZER_ON to 0 if preferred; set the experiment variables var_exp1 and var_exp2 according to the given names above; set the cell arrays as preferred. Run "set_parameters.m" to update. Hit 'Run' in GUI. Results will be stored in /experiment_results directory. To plot results right after the experiment check the 'Plots' checkbox. To plot the results of previous experiments, load the file in /experiment_results directory. Struct variable with name 'results_data' contains all the experiments of that session. Use ">>plot_results(results_data.exp1,'my exp1')" to see results. The results can also be visualized in 3D by passing output vector of integration and parameters to  ">>visualize_forwardDyn(results_data.exp1.xout,results_data.exp1.params)".

-t-

