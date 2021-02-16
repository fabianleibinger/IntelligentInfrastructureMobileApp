import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/events/addvehicle.dart';
import 'package:parkingapp/bloc/events/deletevehicle.dart';
import 'package:parkingapp/bloc/events/resetvehicles.dart';
import 'package:parkingapp/bloc/events/updatevehicle.dart';
import 'package:parkingapp/bloc/events/vehicleevent.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/bloc/events/setvehicles.dart';

class VehicleBloc extends Bloc<VehicleEvent, List<Vehicle>> {
  VehicleBloc(List<Vehicle> initialState) : super(initialState);

  List<Vehicle> get initialState => List<Vehicle>();

  @override
  Stream<List<Vehicle>> mapEventToState(VehicleEvent event) async* {
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
      newState.replaceRange(newState.indexOf(event.eventVehicle),
          newState.indexOf(event.eventVehicle), {event.eventVehicle});
      yield newState;
    }

    /*switch (event is) {
      case SetVehicles:
        List<Vehicle> newState = List.from(state);
        if (event.vehicle != null) {
          newState.add(event.vehicle);
        }
        yield newState;
        break;
      case EventType.delete:
        List<Vehicle> newState = List.from(state);
        newState.removeAt(event.vehicleIndex);
        yield newState;
        break;
      case EventType.update:
        List<Vehicle> newState = List.from(state);
        Iterable<Vehicle> newIterable = [event.vehicle];
        newState.replaceRange(
            event.vehicleIndex, event.vehicleIndex, newIterable);
        yield newState;
        break;
      default:
        throw Exception('Event not found $event');
    }*/
  }
}
