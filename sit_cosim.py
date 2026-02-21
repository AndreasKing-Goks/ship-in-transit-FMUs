# MANDATORY TO LOCATE THE .dll FILES
import os, sys
from pathlib import Path

dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import ctypes

from libcosimpy.CosimExecution import CosimExecution
from libcosimpy.CosimSlave import CosimLocalSlave
from libcosimpy.CosimManipulator import CosimManipulator
from libcosimpy.CosimObserver import CosimObserver
from libcosimpy.CosimEnums import CosimVariableType

from utils import ShipDraw, ship_snap_shot, compile_ship_params
import time


def GetVariableIndex(variables, name):
    try:
        index = 0
        for var in variables:
            if var.name == str.encode(name):
                return index
            index +=1
    except:
        raise Exception("Could not locate the variable %s in the model" %(name))


def GetVariableInfo(variables, name):
    try:
        index = GetVariableIndex(variables,name)
        vr = variables[index].reference
        var_type = CosimVariableType(variables[index].type)
        return vr, var_type
    except:
        raise Exception("Could not locate the variable %s in the model" %(name))


class ObserverStruct:
    var = None
    var_vr = None
    var_type = None
    slave = None
    variable = None


class CoSimInstance:
    '''
        instanceName is the name of the co-simulation instance (str)
        stopTime is the stop time of the co-simulation (seconds)
        stepSize is the macro step size of the co-simulation (seconds)
    '''
    def __init__(self, instanceName: str="simulation", stopTime: float=1.0, stepSize: float = 0.01):
        self.instanceName = instanceName
        self.stopTime = int(stopTime*1e9)
        self.stepSize = int(stepSize*1e9)
        self.time = 0

        self.observer_time_series_struct = {}
        self.observer_time_series_label = {}

        self.slaves = {}
        self.slaves_index = {}
        self.slaves_variables = {}

        self.slave_input_var = []
        self.slave_input_name = []

        self.slave_output_var = []
        self.slave_output_name = []

        self.execution = CosimExecution.from_step_size(self.stepSize)
        self.manipulator = CosimManipulator.create_override()
        self.execution.add_manipulator(self.manipulator)

        self.observer_time_series = CosimObserver.create_time_series(buffer_size=int(self.stopTime/self.stepSize))
        self.execution.add_observer(self.observer_time_series)

        self.observer_last_value = CosimObserver.create_last_value()
        self.execution.add_observer(self.observer_last_value)

        self.first_plot = True

        self.fromExternalSlaveName = []
        self.fromExternalSlaveVar = []
        self.fromExternalSlaveFunc = []
    

    def AddObserverTimeSeries(self, name: str, slaveName: str, variable: str):
        try:
            self.observer_time_series_struct[name]             = ObserverStruct()
            self.observer_time_series_struct[name].slave       = slaveName
            self.observer_time_series_struct[name].var         = variable
            self.observer_time_series_struct[name].var_vr, self.observer_time_series_struct[name].var_type = GetVariableInfo(self.slaves_variables[slaveName],
                                                                                                               variable)
            self.observer_time_series.start_time_series(self.slaves_index[slaveName],
                                                                   value_reference=self.observer_time_series_struct[name].var_vr,
                                                                   variable_type=self.observer_time_series_struct[name].var_type)
        except Exception as error:
            print("An error occured while adding an observer: ", name, "-",slaveName, "-",variable,": ", type(error).__name__, "-", error)
            sys.exit(1)
         
            
    def AddObserverTimeSeriesWithLabel(self, name: str, slaveName: str, variable: str, var_label: str=None):
        try:
            self.observer_time_series_struct[name]             = ObserverStruct()
            self.observer_time_series_struct[name].slave       = slaveName
            self.observer_time_series_struct[name].var         = variable
            self.observer_time_series_struct[name].var_vr, self.observer_time_series_struct[name].var_type = GetVariableInfo(self.slaves_variables[slaveName],
                                                                                                               variable)
            self.observer_time_series.start_time_series(self.slaves_index[slaveName],
                                                                   value_reference=self.observer_time_series_struct[name].var_vr,
                                                                   variable_type=self.observer_time_series_struct[name].var_type)
            self.observer_time_series_label[name]               = var_label
        except Exception as error:
            print("An error occured while adding an observer: ", name, "-",slaveName, "-",variable,": ", type(error).__name__, "-", error)
            sys.exit(1)


    def GetObserverTimeSeries(self, name: str, from_step: int = 0):

        try:
            sample_count = np.int64(self.stopTime / self.stepSize)
            vr = self.observer_time_series_struct[name].var_vr
            var_type = self.observer_time_series_struct[name].var_type
            slave = self.slaves_index[self.observer_time_series_struct[name].slave]
            if var_type == CosimVariableType.REAL:
                time_points, step_numbers, samples = self.observer_time_series.time_series_real_samples(slave_index = slave,
                                                                                         value_reference = vr,
                                                                                         sample_count = sample_count,
                                                                                         from_step = from_step)
                time_seconds = [x*1e-9 for x in time_points]
                return time_seconds, step_numbers, samples
            elif var_type == CosimVariableType.BOOLEAN:
                time_points, step_numbers, samples = self.observer_time_series.time_series_boolean_samples(slave_index = slave,
                                                                                            value_reference = vr,
                                                                                            sample_count = sample_count,
                                                                                            from_step = from_step)
                time_seconds = [x*1e-9 for x in time_points]
                return time_seconds, step_numbers, samples
            elif var_type == CosimVariableType.INTEGER:
                time_points, step_numbers, samples = self.observer_time_series.time_series_integer_samples(slave_index = slave,
                                                                                            value_reference = vr,
                                                                                            sample_count = sample_count,
                                                                                            from_step = from_step)
                time_seconds = [x*1e-9 for x in time_points]
                return time_seconds, step_numbers, samples
            else:
                time_points, step_numbers, samples = self.observer_time_series.time_series_string_samples(slave_index = slave,
                                                                                           value_reference = vr,
                                                                                           sample_count = sample_count,
                                                                                           from_step = from_step)
                time_seconds = [x*1e-9 for x in time_points]
                return time_seconds, step_numbers, samples
        except Exception as error:
            print("An error occured while obtaining a time series: ", name," - ", type(error).__name__, "-", error)
            sys.exit(1)


    def PlotTimeSeries(self, separate_plots: bool = False, create_window: bool = True, create_title: bool = False, show: bool = True, legend: bool = True, show_instance_name: bool=False):
        for key in self.observer_time_series_struct:
            time_points, step_number, samples = self.GetObserverTimeSeries(key)
            if create_window:
                if self.first_plot:
                    plt.figure()
                    self.first_plot = False
                else:
                    if separate_plots:
                        plt.legend()
                        plt.grid()
                        plt.xlabel("Time [s]")
                        plt.ylabel(self.observer_time_series_label[key])
                        if create_title:
                            plt.title("Time series form co-simulation instance \"%s\"" %(self.instanceName))
                        plt.figure()

            label = str(key)
            if show_instance_name:
                label = self.instanceName + ": " + str(key)
    
            plt.plot(time_points, samples, label=label)
        if legend:
            plt.legend()
        plt.xlabel("Time [s]")
        plt.ylabel(self.observer_time_series_label[key])
        if create_title:
            plt.title("Time series form co-simulation instance \"%s\"" %(self.instanceName))
        plt.grid(True)
        if show:
            plt.show()
          
            
    def JoinPlotTimeSeries(self, key_group_list, create_title: bool = False, legend: bool = True, show_instance_name: bool=False, show_separately: bool=False, show=True):
        for key_group in key_group_list:
            struct_time_points  = []
            struct_step_number  = []
            struct_samples      = []
            struct_lables       = []
            
            for key in key_group:
                time_points, step_number, samples = self.GetObserverTimeSeries(key)
                struct_time_points.append(time_points)
                struct_step_number.append(step_number)
                struct_samples.append(samples)
                lable = str(key)
                if show_instance_name:
                    lable = self.instanceName + ": " + str(key)
                struct_lables.append(lable)
            
            plt.figure(figsize=(9,7))
            for i in range(len(key_group)):
                plt.plot(struct_time_points[i], struct_samples[i], label=struct_lables[i])
            if legend:
                plt.legend(fontsize=8)
            plt.grid()
            plt.xticks(fontsize=8)
            plt.yticks(fontsize=8)
            plt.xlabel("Time [s]", fontsize=9)
            plt.ylabel(self.observer_time_series_label[key_group[0]], fontsize=9)
            if create_title:
                plt.title("Time series form co-simulation instance \"%s\"" %(self.instanceName))
            # plt.tight_layout()
            if show_separately and show:
                plt.show()

        if not show_separately and show:
            plt.show()


    def AddSlave(self, path: str, name: str):
        try:
            self.slaves[name] = CosimLocalSlave(fmu_path=path, instance_name = name)
            self.slaves_index[name] = self.execution.add_local_slave(local_slave = self.slaves[name])
            self.slaves_variables[name] = self.execution.slave_variables(slave_index = self.slaves_index[name])
        except Exception as error:
            print("An error occured while adding a slave: ", name, "-",path,": ", type(error).__name__, "-", error)
            sys.exit(1)


    def AddSlaveConnection(self, slaveInputName: str, slaveInputVar: str, slaveOutputName: str, slaveOutputVar: str):
        try:
            self.slave_input_name.append(slaveInputName)
            self.slave_input_var.append(slaveInputVar)
            self.slave_output_name.append(slaveOutputName)
            self.slave_output_var.append(slaveOutputVar)
        except Exception as error:
            print("An error occured while adding a connection: ", slaveInputName, ".",slaveInputVar, " = ",slaveOutputName, ".", slaveOutputVar,": ", type(error).__name__, "-", error)
            sys.exit(1)


    def GetLastValue(self, slaveName: str, slaveVar: str):
        try:
            out_vr, out_type = GetVariableInfo(self.slaves_variables[slaveName], slaveVar)
            if out_type == CosimVariableType.REAL:
                return self.observer_last_value.last_real_values(slave_index = self.slaves_index[slaveName], 
                                                                 variable_references = [out_vr])[0]
            if out_type == CosimVariableType.BOOLEAN:
                return self.observer_last_value.last_boolean_values(slave_index = self.slaves_index[slaveName], 
                                                                 variable_references = [out_vr])[0]
            if out_type == CosimVariableType.INTEGER:
                return self.observer_last_value.last_integer_values(slave_index = self.slaves_index[slaveName], 
                                                                 variable_references = [out_vr])[0]
            if out_type == CosimVariableType.STRING:
                return self.observer_last_value.last_string_values(slave_index = self.slaves_index[slaveName], 
                                                             variable_references = [out_vr])[0]
        except Exception as error:
            print("An error occured while obtaing last value: ", slaveName, ".", slaveVar, ": ", type(error).__name__, "-", error)
            sys.exit(1)


    def CoSimManipulate(self):
        for i in range(0,len(self.slave_input_name)):
            try:
                out_vr, out_type = GetVariableInfo(self.slaves_variables[self.slave_output_name[i]], self.slave_output_var[i])
                out_val = [self.GetLastValue(slaveName = self.slave_output_name[i],
                                            slaveVar = self.slave_output_var[i])]

                if out_type == CosimVariableType.REAL:
                    in_vr, in_type = GetVariableInfo(self.slaves_variables[self.slave_input_name[i]], self.slave_input_var[i])
                    if out_type == in_type:
                        self.manipulator.slave_real_values(self.slaves_index[self.slave_input_name[i]], [in_vr], out_val)
                elif out_type == CosimVariableType.BOOLEAN:
                    in_vr, in_type = GetVariableInfo(self.slaves_variables[self.slave_input_name[i]], self.slave_input_var[i])
                    if out_type == in_type:
                        self.manipulator.slave_boolean_values(self.slaves_index[self.slave_input_name[i]], [in_vr], out_val)
                elif out_type == CosimVariableType.INTEGER:
                    in_vr, in_type = GetVariableInfo(self.slaves_variables[self.slave_input_name[i]], self.slave_input_var[i])
                    if out_type == in_type:
                        self.manipulator.slave_integer_values(self.slaves_index[self.slave_input_name[i]], [in_vr], out_val)
                else:
                    in_vr, in_type = GetVariableInfo(self.slaves_variables[self.slave_input_name[i]], self.slave_input_var[i])
                    if out_type == in_type:
                        self.manipulator.slave_string_values(self.slaves_index[self.slave_input_name[i]], [in_vr], out_val)
            except Exception as error:
                print("An error occured during signal manipulation: ", self.slave_input_name[i],".",self.slave_input_var[i]," = ",self.slave_output_name[i],".", self.slave_output_var[i], " :", type(error).__name__, "-", error)
                sys.exit(1)
    
    
    def SingleVariableManipulation(self, slaveName: str, slaveVar: str, value):    
        try:
            var_vr, var_type = GetVariableInfo(self.slaves_variables[slaveName], slaveVar)
            if var_type == CosimVariableType.REAL:
                self.manipulator.slave_real_values(slave_index=self.slaves_index[slaveName], 
                                                   variable_references=[var_vr], values=[value])

            if var_type == CosimVariableType.BOOLEAN:
                self.manipulator.slave_boolean_values(slave_index=self.slaves_index[slaveName], 
                                                      variable_references=[var_vr], values=[value])

            if var_type == CosimVariableType.INTEGER:
                self.manipulator.slave_integer_values(slave_index=self.slaves_index[slaveName], 
                                                      variable_references=[var_vr], values=[value])

            if var_type == CosimVariableType.STRING:
                self.manipulator.slave_string_values(slave_index=self.slaves_index[slaveName], 
                                                     variable_references=[var_vr], values=[value])
        except Exception as error:
            print("An error occured during single variable manipulation: ", slaveName,".", slaveVar, " = ", value, ": ", type(error).__name__, "-", error)
            sys.exit(1)


    def AddInputFromExternal(self, slaveName: str, slaveVar: str, func):    
        try:
            self.fromExternalSlaveName.append(slaveName)
            self.fromExternalSlaveVar.append(slaveVar)
            self.fromExternalSlaveFunc.append(func)

        except Exception as error:
            print("An error occured while setting an external input: ", slaveName, ".", slaveVar, ": ", type(error).__name__, "-", error)
            sys.exit(1)


    def SetInputFromExternal(self):    
        for i in range(0,len(self.fromExternalSlaveName)):
            try:
                var_vr, var_type = GetVariableInfo(self.slaves_variables[self.fromExternalSlaveName[i]], self.fromExternalSlaveVar[i])
                val =[self.fromExternalSlaveFunc[i]()]
                if var_type == CosimVariableType.REAL:
                    self.manipulator.slave_real_values(self.slaves_index[self.fromExternalSlaveName[i]], [var_vr], val)

                if var_type == CosimVariableType.BOOLEAN:
                    self.manipulator.slave_boolean_values(self.slaves_index[self.fromExternalSlaveName[i]], [var_vr], val)

                if var_type == CosimVariableType.INTEGER:
                    self.manipulator.slave_integer_values(self.slaves_index[self.fromExternalSlaveName[i]], [var_vr], val)

                if var_type == CosimVariableType.STRING:
                    self.manipulator.slave_string_values(self.slaves_index[self.fromExternalSlaveName[i]], [var_vr], val)
            except Exception as error:
                print("An error occured during signal manipulation from external: ", self.fromExternalSlaveName[i],".",self.fromExternalSlaveVar[i], " = ", val, ": ", type(error).__name__, "-", error)
                sys.exit(1)


    def SetInitialValue(self, slaveName: str, slaveVar: str, initValue):
        try:
            var_vr, var_type = GetVariableInfo(self.slaves_variables[slaveName], slaveVar)
            if var_type == CosimVariableType.REAL:
                self.execution.real_initial_value(slave_index = self.slaves_index[slaveName],
                                                  variable_reference = var_vr, value = initValue)

            if var_type == CosimVariableType.BOOLEAN:
                self.execution.boolean_initial_value(slave_index = self.slaves_index[slaveName],
                                                  variable_reference = var_vr, value = initValue)

            if var_type == CosimVariableType.INTEGER:
                self.execution.integer_initial_value(slave_index = self.slaves_index[slaveName],
                                                  variable_reference = var_vr, value = initValue)

            if var_type == CosimVariableType.STRING:
                self.execution.string_initial_value(slave_index = self.slaves_index[slaveName],
                                                  variable_reference = var_vr, value = initValue)
        except Exception as error:
            print("An error occured while setting an initial value: ", slaveName, ".", slaveVar, " = ", initValue, ": ", type(error).__name__, "-", error)
            sys.exit(1)
         
            
    def SetInitialValues(self, slaveName: str, params: dict):
        for var_name, value in params.items():
            self.SetInitialValue(slaveName, var_name, value)


    def PreSolverFunctionCall(self):
        pass


    def PostSolverFunctionCall(self):
        pass


    def Simulate(self):
        while self.time < self.stopTime:
            self.CoSimManipulate()
            self.SetInputFromExternal()
            self.PreSolverFunctionCall()
            self.execution.step()
            self.PostSolverFunctionCall()
            self.time +=self.stepSize
            

class ShipInTransitCoSimulation(CoSimInstance):
    '''
        This class is the extension of the CoSimInstance class
        especially built for running the FMU-based Ship In Transit Simulator
    '''
    def __init__(self,
                 instanceName   : str     = "simulation",
                 stopTime       : float   = 1.0, 
                 stepSize       : float   = 0.01):
        super().__init__(instanceName, stopTime, stepSize)
        
        # For colav_active printing flag
        self.print_col_msg = False


    def ship_slave(self, prefix: str, block: str) -> str:
        return f"{prefix}__{block}"
    
    
    def add_ship(self,
                 ship_configs,
                 ROOT: Path ):
        
        # Store ship configs as the class attribute
        self.ship_configs = ship_configs
        
        for ship_config in ship_configs:
            prefix              = ship_config.get("id")
            role                = ship_config.get("role")
            enable_colav        = ship_config.get("enable_colav")
            SHIP_BLOCKS         = ship_config.get("SHIP_BLOCKS")
            SHIP_CONNECTIONS    = ship_config.get("SHIP_CONNECTIONS")
            SHIP_OBSERVERS      = ship_config.get("SHIP_OBSERVERS")
            fmu_params          = compile_ship_params(ship_config)
            
            # Add slaves
            for block, relpath in SHIP_BLOCKS:
                self.AddSlave(
                    name=self.ship_slave(prefix, block),
                    path=str(ROOT / relpath)
                )

            # Set initial values
            for block, p in fmu_params.items():
                self.SetInitialValues(slaveName=self.ship_slave(prefix, block), params=p)

            # Add internal connections
            for in_block, in_var, out_block, out_var in SHIP_CONNECTIONS:
                self.AddSlaveConnection(
                    slaveInputName=self.ship_slave(prefix, in_block),
                    slaveInputVar=in_var,
                    slaveOutputName=self.ship_slave(prefix, out_block),
                    slaveOutputVar=out_var,
                )
            
            # Add COLAV FMU for the ship that has it
            if enable_colav:
                # All other ships (exclude self)
                targets = [sc for sc in ship_configs if sc.get("id") != prefix]

                for idx, tgt in enumerate(targets, start=1):
                    tgt_prefix = tgt.get("id")

                    self.AddSlaveConnection(
                        slaveInputName=self.ship_slave(prefix, "COLAV"),
                        slaveInputVar=f"tar_{idx}_north",
                        slaveOutputName=self.ship_slave(tgt_prefix, "SHIP_MODEL"),
                        slaveOutputVar="north",
                    )
                    self.AddSlaveConnection(
                        slaveInputName=self.ship_slave(prefix, "COLAV"),
                        slaveInputVar=f"tar_{idx}_east",
                        slaveOutputName=self.ship_slave(tgt_prefix, "SHIP_MODEL"),
                        slaveOutputVar="east",
                    )
                    self.AddSlaveConnection(
                        slaveInputName=self.ship_slave(prefix, "COLAV"),
                        slaveInputVar=f"tar_{idx}_yaw_angle",
                        slaveOutputName=self.ship_slave(tgt_prefix, "SHIP_MODEL"),
                        slaveOutputVar="yaw_angle_rad",
                    )
                    self.AddSlaveConnection(
                        slaveInputName=self.ship_slave(prefix, "COLAV"),
                        slaveInputVar=f"tar_{idx}_measured_speed",
                        slaveOutputName=self.ship_slave(tgt_prefix, "SHIP_MODEL"),
                        slaveOutputVar="total_ship_speed",
                    )
                    
            
            # Add observers (namespaced keys)
            for block, var, label in SHIP_OBSERVERS:
                self.AddObserverTimeSeriesWithLabel(
                    name=f"{prefix}.{var}",
                    slaveName=self.ship_slave(prefix, block),
                    variable=var,
                    var_label=label
                )

            # Add ShipDraw class
            draw_attr = f"{prefix}_draw"
            setattr(self, 
                    draw_attr, 
                    ShipDraw(fmu_params["SHIP_MODEL"].get("length_of_ship"), fmu_params["SHIP_MODEL"].get("width_of_ship")))
            
        
    def PreSolverFunctionCall(self):
        pass


    def PostSolverFunctionCall(self):
        """
        Stop the simulator once the stop flag is received
        """
        # Get the reach end point and collision flag for all assets
        rep_flags       = []
        collision_flags = []
        
        for ship_config in self.ship_configs:
            prefix = ship_config.get("id")
            rep_flag  = self.GetLastValue(slaveName=self.ship_slave(prefix, "MISSION_MANAGER"), 
                                         slaveVar="reach_wp_end")
            rep_flags.append(rep_flag)
            
            if ship_config.get("enable_colav"):
                colav_active = self.GetLastValue(slaveName=self.ship_slave(prefix, "COLAV"), 
                                         slaveVar="colav_active")
                collision_flag = self.GetLastValue(slaveName=self.ship_slave(prefix, "COLAV"), 
                                         slaveVar="ship_collision")
                collision_flags.append(collision_flag)
                
                if colav_active and (not collision_flag) and (not self.print_col_msg):
                    print(f"{prefix}_COLAV is active!")
                    self.print_col_msg = True
                elif (not colav_active) and self.print_col_msg:
                    print(f"{prefix}_COLAV is deactived!")
                    self.print_col_msg = False
                elif collision_flag:
                    print(f"{prefix} collides!")
        
        all_ship_reaches_end_point = np.all(np.array(rep_flags))
        any_ship_collides          = np.any(np.array(collision_flags))
        
        # Conclude the stop flag
        self.stop = all_ship_reaches_end_point or any_ship_collides
        

    def Simulate(self):
        """
        Output simulation time
        """
        # Start timer
        start_time = time.perf_counter()
        
        while self.time < self.stopTime: 
            # Simulate
            self.CoSimManipulate()
            self.SetInputFromExternal()
            self.PreSolverFunctionCall()
            self.execution.step()
            self.PostSolverFunctionCall()
            
            # Integrate or stop the simulator
            if not self.stop:
                self.time +=self.stepSize
            else:
                break
        
        # Stop timer
        end_time = time.perf_counter()
        
        # Compute elapsed time
        elapsed_time = end_time -start_time
        
        print(f"Simulation took {elapsed_time:.6f} seconds.")
        
            
    def _get_ship_timeseries(self, ship_id: str, var: str):
        key = f"{ship_id}.{var}"
        t, step, samples = self.GetObserverTimeSeries(key)
        return np.asarray(t), np.asarray(step), np.asarray(samples)

    
    def PlotFleetTrajectory(
        self,
        ship_ids=None,          # None = plot all ships in self.ship_configs
        show=True,
        mode="quick",
        block=True,
        fig_width=10.0,
        every_n=25,
        margin_frac=0.08,
        equal_aspect=True,
        save_path=None,
        plot_routes=True,
        plot_waypoints=True,
        plot_outlines=True,
    ):
        # Pick ships
        if ship_ids is None:
            ship_ids = [sc.get("id") for sc in self.ship_configs]
        ship_ids = [sid for sid in ship_ids if sid is not None]

        # Plot configurations (reuse yours)
        if mode == "paper":
            own_lw, route_lw, ship_lw = 2.4, 1.8, 2.0
            title_fs, label_fs, tick_fs, legend_fs = 12, 11, 10, 9
            grid_alpha, roa_alpha, dpi = 0.4, 0.25, 500
        elif mode == "quick":
            own_lw, route_lw, ship_lw = 1.2, 1.0, 1.6
            title_fs, label_fs, tick_fs, legend_fs = 9, 8, 7, 7
            grid_alpha, roa_alpha, dpi = 0.2, 0.15, 110
        else:
            raise ValueError("mode must be 'quick' or 'paper'")

        every_n = max(1, int(every_n))

        # Figure
        fig, ax = plt.subplots(figsize=(fig_width, fig_width), dpi=dpi)

        # simple palette (you can replace with your own, incl. colorblind-safe)
        palette = ["#0c3c78", "#d90808", "#2a9d8f", "#f4a261", "#6a4c93", "#264653"]

        # track global extents (route or traj)
        all_x, all_y = [], []

        # per-ship plotting
        for k, sid in enumerate(ship_ids):
            color = palette[k % len(palette)]

            # --- time series ---
            _, _, north = self._get_ship_timeseries(sid, "north")
            _, _, east  = self._get_ship_timeseries(sid, "east")
            _, _, yaw   = self._get_ship_timeseries(sid, "yaw_angle_rad")
            
            n = min(len(north), len(east), len(yaw))
            north, east, yaw = north[1:n], east[1:n], yaw[1:n]

            all_x.append(east)
            all_y.append(north)
            
            # --- trajectory ---
            ax.plot(east, north, lw=own_lw, color=color, label=f"{sid} trajectory")

            # --- route per ship (from config) ---
            if plot_routes:
                print("Drawing routes ...")
                cfg = next((sc for sc in self.ship_configs if sc.get("id") == sid), None)
                if cfg and "route" in cfg:
                    rN = cfg["route"].get("north", [])
                    rE = cfg["route"].get("east", [])
                    if len(rN) and len(rE):
                        ax.plot(rE, rN, lw=route_lw, ls="--", color=color, alpha=0.7,
                                label=f"{sid} route")
                        if plot_waypoints:
                            ax.scatter(rE, rN,
                                    s=18 if mode == "quick" else 30,
                                    marker="x", color=color)

                            # RoA circles
                            ra = cfg["fmu_params"].get("mission_manager", {}).get("ra", None)
                            if ra is not None:
                                print("Drawing RoA ...")
                                for n_wp, e_wp in zip(rN[1:], rE[1:]):
                                    circ = patches.Circle(
                                        (e_wp, n_wp),
                                        radius=ra,
                                        fill=True,
                                        color=color,
                                        alpha=roa_alpha
                                    )
                                    ax.add_patch(circ)

            # --- ship outlines ---
            if plot_outlines:
                print("Drawing ship outlines ...")
                idx = np.arange(0, n, every_n)
                draw_attr = f"{sid}_draw"
                draw = getattr(self, draw_attr)
                for i in idx[::6]:  # extra subsample for speed
                    x, y = draw.local_coords()
                    x_ned, y_ned = draw.rotate_coords(x, y, yaw[i])
                    x_tr,  y_tr  = draw.translate_coords(x_ned, y_ned, north[i], east[i])
                    ax.plot(y_tr, x_tr, lw=ship_lw, color=color, alpha=0.6)

        # Zoom (global)
        X = np.concatenate(all_x) if len(all_x) else np.array([0.0, 1.0])
        Y = np.concatenate(all_y) if len(all_y) else np.array([0.0, 1.0])

        x_min, x_max = float(np.min(X)), float(np.max(X))
        y_min, y_max = float(np.min(Y)), float(np.max(Y))

        dx = max(1e-9, x_max - x_min)
        dy = max(1e-9, y_max - y_min)

        ax.set_xlim(x_min - margin_frac * dx, x_max + margin_frac * dx)
        ax.set_ylim(y_min - margin_frac * dy, y_max + margin_frac * dy)

        # Styling
        ax.set_title("Fleet trajectories", fontsize=title_fs, pad=4)
        ax.set_xlabel("East position (m)", fontsize=label_fs)
        ax.set_ylabel("North position (m)", fontsize=label_fs)

        ax.tick_params(axis="both", which="major", labelsize=tick_fs, length=3)
        ax.grid(True, which="major", linewidth=0.6, alpha=grid_alpha)

        leg = ax.legend(fontsize=legend_fs, frameon=True, framealpha=0.75,
                        borderpad=0.4, handlelength=2.2, loc="lower center")
        leg.get_frame().set_linewidth(0.6)

        ax.ticklabel_format(style='sci', axis='both', scilimits=(0,0))
        ax.xaxis.get_offset_text().set_fontsize(tick_fs-1)
        ax.yaxis.get_offset_text().set_fontsize(tick_fs-1)

        if equal_aspect:
            ax.set_aspect("equal", adjustable="box")

        fig.tight_layout(pad=0.05)

        if save_path:
            fig.savefig(save_path, dpi=dpi, bbox_inches="tight")

        if show:
            print("Plot is finished!")
            plt.show(block=block)

        return fig, ax

