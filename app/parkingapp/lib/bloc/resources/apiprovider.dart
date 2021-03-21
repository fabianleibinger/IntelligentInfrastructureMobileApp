import 'dart:async';
import 'dart:io';
import 'package:http/http.dart' as http;
import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'dart:convert';
import 'package:parkingapp/models/classes/vehicle.dart';

/// A Service that defines the calls to the backend server
class ApiProvider {
  /// The Backend Server Address.
  //TODO new url 192.168.4.1
  static final String _serverUrl = 'http://10.0.2.2';
  static final String _serverPort = ':2525';

  /// The HTTP status codes.
  static final int httpGetStatusCodeSuccess = 200;
  static final int httpPostStatusCodeSuccess = 200;

  /// The HTTP Header specifications.
  static final String _httpPostHeaderFirst = 'Content-Type';
  static final String _httpPostHeaderSecond = 'application/json; charset=UTF-8';

  /// The time span that triggers a time out.
  static final Duration _timeOutAfter = Duration(seconds: 10);

  /// Returns connection details.
  /// ['IP', 'Port', 'Parking garage']
  static Future<Map<String, dynamic>> connect() async {
    return httpGet(
        _serverUrl + _serverPort + '/connect', 'Failed to connect to server');
  }

  /// Returns parking garage capacities.
  /// ['total', 'electric', 'electric_fast', 'electric_inductive']
  static Future<Map<String, dynamic>> getCapacity() async {
    return httpGet(
        _serverUrl + _serverPort + '/capacities', 'Failed to get capacities');
  }

  /// Returns free parking garage capacities.
  /// ['free_total', 'free_electric', 'free_electric_fast', 'free_electric_inductive']
  static Future<Map<String, dynamic>> getFreeCapacity() async {
    return httpGet(
        _serverUrl + _serverPort + '/free', 'Failed to get free capacities');
  }

  /// Returns free total parking spots.
  /// ['free_total']
  static Future<Map<String, dynamic>> getFreeParkingSpots() async {
    return httpGet(_serverUrl + _serverPort + '/free/total',
        'Failed to get free total parking spots');
  }

  /// Returns free chargeable parking spots.
  /// ['free_electric']
  static Future<Map<String, dynamic>> getFreeChargeableParkingSpots() async {
    return httpGet(_serverUrl + _serverPort + '/free/electric',
        'Failed to get free chargeable parking spots');
  }

  /// Tries to park in [vehicle].
  /// Returns ['parking', 'longitude', 'latitude', 'load_vehicle'].
  static Future<Map<String, dynamic>> parkIn(Vehicle vehicle) async {
    return httpPost(_serverUrl + _serverPort + '/parkIn',
        _chooseParkInBody(vehicle), 'Failed to park in vehicle');
  }

  /// Tries to park out or cancel park in for [vehicle].
  /// Returns ['parking', 'longitude', 'latitude'].
  static Future<Map<String, dynamic>> parkOut(Vehicle vehicle) async {
    return httpPost(_serverUrl + _serverPort + "/parkOut",
        _chooseParkOutBody(vehicle), 'Failed to park out vehicle');
  }

  /// Returns the position of the [vehicle].
  /// ['longitude', 'latitude', 'moving', 'reached_position']
  static Future<Map<String, dynamic>> getPosition(Vehicle vehicle) async {
    return httpPost(_serverUrl + _serverPort + "/getPosition",
        _chooseGetPositionBody(vehicle), 'Failed to get position of vehicle');
  }

  /// Returns the response body of the HTTP Get request to [url].
  ///
  /// Throws an [HttpException] with the [failureText].
  static Future<Map<String, dynamic>> httpGet(
      String url, String failureText) async {
    final response = await http.get(url).timeout(_timeOutAfter);
    if (response.statusCode == httpGetStatusCodeSuccess) {
      final Map result = json.decode(response.body);
      print(result.entries.toString());
      return result;
    } else {
      throw HttpException(failureText);
    }
  }

  /// Returns the response body of the HTTP Post request to [url].
  ///
  /// Throws an [HttpException] with the [failureText].
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
      throw HttpException(failureText);
    }
  }

  /// Returns the correct HTTP Post body for a [vehicle] used in [ApiProvider.parkIn(vehicle)].
  static Map<String, dynamic> _chooseParkInBody(Vehicle vehicle) {
    if (vehicle.runtimeType == ChargeableVehicle) {
      return _parkInBodyChargeableVehicle(vehicle);
    } else {
      return _parkInBodyStandardVehicle(vehicle);
    }
  }

  /// Returns the HTTP Post body for [chargeableVehicle] park in requests.
  static Map<String, dynamic> _parkInBodyChargeableVehicle(
      ChargeableVehicle chargeableVehicle) {
    return {
      "id": chargeableVehicle.databaseId,
      "length": chargeableVehicle.length,
      "width": chargeableVehicle.width,
      "turning_radius": chargeableVehicle.turningCycle,
      "dist_rear_axle_numberplate": chargeableVehicle.distRearAxleLicensePlate,
      "charge_type": "electric",
      "number_plate": chargeableVehicle.licensePlate,
      "near_exit": chargeableVehicle.nearExitPreference,
      "parking_card": chargeableVehicle.parkingCard,
      "load": chargeableVehicle.doCharge,
      "charge_service_provider": chargeableVehicle.chargingProvider
    };
  }

  /// Returns the HTTP Post body for [standardVehicle] park in requests.
  static Map<String, dynamic> _parkInBodyStandardVehicle(
      StandardVehicle standardVehicle) {
    return {
      "id": standardVehicle.databaseId,
      "length": standardVehicle.length,
      "width": standardVehicle.width,
      "turning_radius": standardVehicle.turningCycle,
      "dist_rear_axle_numberplate": standardVehicle.distRearAxleLicensePlate,
      "number_plate": standardVehicle.licensePlate,
      "near_exit": standardVehicle.nearExitPreference,
      "parking_card": standardVehicle.parkingCard
    };
  }

  /// Returns the correct HTTP Post body for a [vehicle] used in [ApiProvider.parkOut(vehicle)].
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

  /// Returns the correct HTTP Post body for a [vehicle] used in [ApiProvider.getPosition(vehicle)].
  static Map<String, dynamic> _chooseGetPositionBody(Vehicle vehicle) {
    return {"id": vehicle.inAppKey, "number_plate": vehicle.licensePlate};
  }
}
