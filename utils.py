# MANDATORY TO LOCATE THE .dll FILES
import os, sys
from pathlib import Path

dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

import matplotlib.pyplot as plt
from libcosimpy.CosimExecution import CosimExecution
from libcosimpy.CosimSlave import CosimLocalSlave
from libcosimpy.CosimManipulator import CosimManipulator
from libcosimpy.CosimObserver import CosimObserver
from libcosimpy.CosimEnums import CosimVariableType
import numpy as np
import ctypes

# UTILS
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

