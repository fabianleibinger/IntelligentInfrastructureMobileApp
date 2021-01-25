import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:parkingapp/bloc/events/vehicleevent.dart';
import 'package:parkingapp/models/classes/vehicle.dart';

class VehicleBloc extends Bloc<VehicleEvent, List<Vehicle>> {
  VehicleBloc(List<Vehicle> initialState) : super(initialState);

  @override
  List<Vehicle> get initialState => List<Vehicle>();

  @override
  Stream<List<Vehicle>> mapEventToState(VehicleEvent event) async* {
    switch (event.eventType) {
      case EventType.add:
        List<Vehicle> newState = List.from(state);
        if (event.vehicle != null) {
          newState.add(event.vehicle);
        }
        yield null;
        break;
      case EventType.delete:
        List<Vehicle> newState = List.from(state);
        newState.removeAt(event.vehicleIndex);
        yield null;
        break;
      case EventType.modify:
        List<Vehicle> newState = List.from(state);
        Iterable<Vehicle> newIterable = [event.vehicle];
        newState.replaceRange(
            event.vehicleIndex, event.vehicleIndex, newIterable);
        yield null;
        break;
      default:
        throw Exception('Event not found $event');
    }
  }
}
