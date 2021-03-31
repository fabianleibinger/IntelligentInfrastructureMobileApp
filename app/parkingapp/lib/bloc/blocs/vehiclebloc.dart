import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/events/addvehicle.dart';
import 'package:parkingapp/bloc/events/deletevehicle.dart';
import 'package:parkingapp/bloc/events/resetvehicles.dart';
import 'package:parkingapp/bloc/events/updatevehicle.dart';
import 'package:parkingapp/bloc/events/vehicleevent.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/bloc/events/setvehicles.dart';

/// stores all the vehicles during runtime as a list
/// accessed via the DataHelper class

class VehicleBloc extends Bloc<VehicleEvent, List<Vehicle>> {
  VehicleBloc(List<Vehicle> initialState) : super(initialState);

  List<Vehicle> get initialState => List<Vehicle>();

  @override
  Stream<List<Vehicle>> mapEventToState(VehicleEvent event) async* {
    // defines the behavior of the specific events
    // these are childs of the abstract class vehicleEvent
    if (event is SetVehicles) {
      yield event.vehicleList;
    } else if (event is AddVehicle) {
      List<Vehicle> newState = List.from(state);
      if (event.eventVehicle != null) {
        newState.add(event.eventVehicle);
      }
      yield newState;
    } else if (event is ResetVehicles) {
      List<Vehicle> newState = List.from(state);
      newState.clear();
      yield newState;
    } else if (event is DeleteVehicle) {
      List<Vehicle> newState = List.from(state);
      newState.removeAt(newState.indexOf(event.eventVehicle));
      yield newState;
    } else if (event is UpdateVehicle) {
      List<Vehicle> newState = List.from(state);
      int index;
      newState.forEach((element) {
        if (element.inAppKey == event.eventVehicle.inAppKey) {
          index = newState.indexOf(element);
        }
      });
      newState.fillRange(index, index, event.eventVehicle);
      yield newState;
    }
  }
}
