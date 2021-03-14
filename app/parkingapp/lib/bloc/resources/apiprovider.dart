import 'dart:async';
import 'package:http/http.dart' as http;
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'dart:convert';
import 'package:parkingapp/models/classes/vehicle.dart';

// defines the calls to the backend and what should happen with the results

class ApiProvider {
  static final int httpGetStatusCodeSuccess = 200;
  static final int httpPostStatusCodeSuccess = 200;

  static final String _serverUrl = 'http://10.0.2.2';
  static final String _serverPort = ':2525';

  static final Duration _timeOutAfter = Duration(seconds: 10);

  // ['IP', 'Port', 'Parking garage']
  //tries to connect to server
  static Future<Map<String, dynamic>> connect() async {
    return httpGet(
        _serverUrl + _serverPort + '/connect', 'Failed to connect to server');
  }

  // ['total', 'electric', 'electric_fast', 'electric_inductive']
  //tries to get parking capacities
  static Future<Map<String, dynamic>> getCapacity() async {
    return httpGet(
        _serverUrl + _serverPort + '/capacities', 'Failed to get capacities');
  }

  // ['free_total', 'free_electric', 'free_electric_fast', 'free_electric_inductive']
  //tries to get free parking capacities
  static Future<Map<String, dynamic>> getFreeCapacity() async {
    return httpGet(
        _serverUrl + _serverPort + '/free', 'Failed to get capacities');
  }

  // ['free_total']
  //tries to get free parking spots
  static Future<Map<String, dynamic>> getFreeParkingSpots() async {
    return httpGet(_serverUrl + _serverPort + '/free/total',
        'Failed to get free parking spots');
  }

  // ['free_electric']
  //tries to get free chargeable parking spots
  static Future<Map<String, dynamic>> getFreeChargeableParkingSpots() async {
    return httpGet(_serverUrl + _serverPort + '/free/electric',
        'Failed to get free chargeable parking spots');
  }

  //http get request that returns the response body
  static Future<Map<String, dynamic>> httpGet(
      String url, String failureText) async {
    final response = await http.get(url).timeout(_timeOutAfter);
    if (response.statusCode == httpGetStatusCodeSuccess) {
      final Map result = json.decode(response.body);
      print(result.entries.toString());
      return result;
    } else {
      throw Exception(failureText);
    }
  }

  //tries to park in vehicle
  static Future<Map<String, dynamic>> parkIn(Vehicle vehicle) async {
    final response = await http
        .post(_serverUrl + _serverPort + "/parkIn",
            headers: <String, String>{
              'Content-Type': 'application/json; charset=UTF-8',
            },
            body: vehicle.runtimeType == ChargeableVehicle
                ? parkInBodyChargeableVehicle(vehicle)
                : parkInBodyStandardVehicle(vehicle))
        .timeout(_timeOutAfter);
    if (response.statusCode == httpPostStatusCodeSuccess) {
      final Map result = json.decode(response.body);
      print(result);
      return result;
    } else {
      throw Exception('Failed to park in');
    }
  }

  //TODO add body
  //tries to park out vehicle or cancel park in
  static Future<Map<String, dynamic>> parkOut(Vehicle vehicle) async {
    final response = await http.get(
      _serverUrl + _serverPort + "/parkOut",
      headers: <String, String>{
        'Content-Type': 'application/json; charset=UTF-8',
      },
    ).timeout(_timeOutAfter);
    if (response.statusCode == httpPostStatusCodeSuccess) {
      final Map result = json.decode(response.body);
      return result;
    } else {
      throw Exception('Failed to park out');
    }
  }

  //TODO add body
  //tries to get parked in confirmation
  static Future<Map<String, dynamic>> getParkedIn(Vehicle vehicle) async {
    final response = await http
        .post(_serverUrl + _serverPort + "/parkedIn",
            headers: <String, String>{
              'Content-Type': 'application/json; charset=UTF-8',
            },
            body: jsonEncode(
                {"id": vehicle.inAppKey, "number_plate": vehicle.licensePlate}))
        .timeout(_timeOutAfter);
    if (response.statusCode == httpPostStatusCodeSuccess) {
      final Map result = json.decode(response.body);
      return result;
    } else {
      throw Exception('Failed to get parked in confirmation');
    }
  }

  //tries to get position of the vehicle
  static Future<Map<String, dynamic>> getPosition(Vehicle vehicle) async {
    final response = await http
        .post(_serverUrl + _serverPort + "/getPosition",
            headers: <String, String>{
              'Content-Type': 'application/json; charset=UTF-8',
            },
            body: jsonEncode(
                {"id": vehicle.inAppKey, "number_plate": vehicle.licensePlate}))
        .timeout(_timeOutAfter);
    if (response.statusCode == httpPostStatusCodeSuccess) {
      final Map result = json.decode(response.body);
      return result;
    } else {
      throw Exception('Failed to get position');
    }
  }

  static String parkInBodyChargeableVehicle(ChargeableVehicle vehicle) {
    String json = jsonEncode(<String, dynamic>{
      "id": vehicle.databaseId,
      "length": vehicle.length,
      "width": vehicle.width,
      "turning_radius": vehicle.turningCycle,
      "dist_rear_axle_numberplate": vehicle.distRearAxleLicensePlate,
      "charge_type": "electric",
      "number_plate": vehicle.licensePlate,
      "near_exit": vehicle.nearExitPreference,
      "parking_card": vehicle.parkingCard,
      "load": vehicle.doCharge,
      "charge_service_provider": vehicle.chargingProvider
    });
    print(json.runtimeType);
    print(json);
    return json;
  }

  static String parkInBodyStandardVehicle(StandardVehicle vehicle) {
    return jsonEncode(<String, dynamic>{
      "id": vehicle.databaseId,
      "length": vehicle.length,
      "width": vehicle.width,
      "turning_radius": vehicle.turningCycle,
      "dist_rear_axle_numberplate": vehicle.distRearAxleLicensePlate,
      "number_plate": vehicle.licensePlate,
      "near_exit": vehicle.nearExitPreference,
      "parking_card": vehicle.parkingCard
    });
  }
}
