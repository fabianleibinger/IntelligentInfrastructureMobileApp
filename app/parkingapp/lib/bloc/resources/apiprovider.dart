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

  //TODO new url 192.168.4.1
  static final String _serverUrl = 'http://10.0.2.2';
  static final String _serverPort = ':2525';

  static final Duration _timeOutAfter = Duration(seconds: 10);

  static final String _httpPostHeaderFirst = 'Content-Type';
  static final String _httpPostHeaderSecond = 'application/json; charset=UTF-8';

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

  //['parking', 'longitude', 'latitude', 'load_vehicle']
  //tries to park in vehicle
  static Future<Map<String, dynamic>> parkIn(Vehicle vehicle) async {
    return httpPost(_serverUrl + _serverPort + '/parkIn',
        _chooseParkInBody(vehicle), 'Failed to park in');
  }

  //['parking', 'longitude', 'latitude']
  //tries to park out vehicle or cancel park in
  static Future<Map<String, dynamic>> parkOut(Vehicle vehicle) async {
    return httpPost(
        _serverUrl + _serverPort + "/parkOut",
        _chooseParkOutBody(vehicle),
        'Failed to park out');
  }

  //['longitude', 'latitude', 'moving', 'reached_position']
  //tries to get position of the vehicle
  static Future<Map<String, dynamic>> getPosition(Vehicle vehicle) async {
    return httpPost(
        _serverUrl + _serverPort + "/getPosition",
        _chooseGetPositionBody(vehicle),
        'Failed to get position');
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

  //http post request that returns the response body
  static Future<Map<String, dynamic>> httpPost(
      String url, Map<String, dynamic> body, String failureText) async {
    final response = await http
        .post(url,
            headers: <String, String>{
              _httpPostHeaderFirst: _httpPostHeaderSecond,
            },
            body: jsonEncode(body))
        .timeout(_timeOutAfter);
    if (response.statusCode == httpGetStatusCodeSuccess) {
      final Map result = json.decode(response.body);
      print(result.entries.toString());
      return result;
    } else {
      throw Exception(failureText);
    }
  }

  //chooses and returns http post body for park in call
  static Map<String, dynamic> _chooseParkInBody(Vehicle vehicle) {
    if (vehicle.runtimeType == ChargeableVehicle) {
      return _parkInBodyChargeableVehicle(vehicle);
    } else {
      return _parkInBodyStandardVehicle(vehicle);
    }
  }

  static Map<String, dynamic> _parkInBodyChargeableVehicle(
      ChargeableVehicle vehicle) {
    return {
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
    };
  }

  static Map<String, dynamic> _parkInBodyStandardVehicle(
      StandardVehicle vehicle) {
    return {
      "id": vehicle.databaseId,
      "length": vehicle.length,
      "width": vehicle.width,
      "turning_radius": vehicle.turningCycle,
      "dist_rear_axle_numberplate": vehicle.distRearAxleLicensePlate,
      "number_plate": vehicle.licensePlate,
      "near_exit": vehicle.nearExitPreference,
      "parking_card": vehicle.parkingCard
    };
  }

  //chooses and returns http post body for park out call
  static Map<String, dynamic> _chooseParkOutBody(Vehicle vehicle) {
    return {
      "id": vehicle.databaseId,
      "number_plate": vehicle.licensePlate,
      "length": vehicle.length,
      "width": vehicle.width,
      "turning_radius": vehicle.turningCycle,
      "dist_rear_axle_numberplate": vehicle.distRearAxleLicensePlate,
    };
  }

  //chooses and returns http post body for park out call
  static Map<String, dynamic> _chooseGetPositionBody(Vehicle vehicle) {
    return {"id": vehicle.inAppKey, "number_plate": vehicle.licensePlate};
  }
}
