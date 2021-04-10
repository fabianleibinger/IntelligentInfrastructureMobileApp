import 'dart:async';

import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/resources/apiprovider.dart';
import 'package:parkingapp/dialogs/noconnectiondialog.dart';
import 'package:parkingapp/models/classes/coordinate.dart';
import 'package:parkingapp/notifications/notifications.dart';
import 'package:parkingapp/dialogs/parkdialogs.dart';
import 'package:parkingapp/dialogs/parkinggarageoccupieddialog.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/routes/routes.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

//how often to retry the park operations
int _errorLimit = 20;

/// Utility class that communicates with the [ApiProvider]
/// to manage park processes for vehicles.
class ParkManager {
  /// Sends the [vehicle] park in request to the [ApiProvider].
  static void parkInRequest(BuildContext context, Vehicle vehicle) {
    if (needsToParkIn(context, vehicle)) {
      if (currentParkingGarage.vehicleSpecificSpotsAvailable(vehicle)) {
        vehicle.setAndUpdateParkIngIn(context, true);
        print(vehicle.name + ' parking in');

        int _errorCount = 0;
        // Try to contact server.
        ApiProvider.parkIn(vehicle).then((value) {
          vehicle.setParkingSpot(value["latitude"], value["longitude"]);

          // Update the position while the vehicle is parking.
          new Timer.periodic(Duration(seconds: 1), (timer) {
            print('ParkIn: updating vehicle ' + vehicle.name);
            ParkManager.updatePosition(vehicle).then((bool parking) {
              _checkAndReactIfParkInFailed(vehicle, timer, context);
              _checkAndReactIfVehicleParkedIn(vehicle, parking, timer, context);
            }).catchError((e) {
              _handleGetPositionErrorForParkInRequest(
                  _errorCount, timer, context);
              return;
            });
          });
        }).catchError((e) => _noParkingSpotsAvailable(context));
      } else
        _noParkingSpotsAvailable(context);
    } else {
      // vehicle is already parked in.
      print('vehicle ' + vehicle.name + ' is already parked in');
    }
  }

  /// Returns if [vehicle] needs to be parked in.
  /// Parking out vehicle can't be parked in.
  static bool needsToParkIn(BuildContext context, Vehicle vehicle) {
    ApiProvider.getPosition(vehicle).then((value) {
      if (value["reached_position"] != null)
        vehicle.setAndUpdateParkedIn(context, value["reached_position"]);
      if (value["parking"] != null)
        vehicle.setAndUpdateParkIngIn(context, value["parking"]);
      print('needs to park in? parked in: ' +
          vehicle.parkedIn.toString() +
          ' parking in: ' +
          vehicle.parkingIn.toString());
    }).catchError((e) {
      print('could not update vehiclePosition');
      vehicle.setAndUpdateParkedIn(context, false);
      vehicle.setAndUpdateParkIngIn(context, false);
    });
    return !vehicle.parkedIn && !vehicle.parkingIn && !vehicle.parkingOut;
  }

  /// Cancels [timer] if park in failed for [vehicle].
  /// Returns [NoConnectionDialog] if [vehicle] didn't cancel park in.
  static Future _checkAndReactIfParkInFailed(
      Vehicle vehicle, Timer timer, BuildContext context) {
    if (!vehicle.parkedIn && !vehicle.parkingIn) {
      timer.cancel();

      // vehicle could not be cancelling park in.
      if (!vehicle.parkingOut) {
        // Park in didn't work: connection to server failed.
        showDialog(
            context: context,
            builder: (context) {
              return NoConnectionDialog.getDialog(context);
            }).then((value) => _setParkedOut(vehicle, context));
      }
    }
    return null;
  }

  /// Cancels [timer] and calls _setParkedIn() for [vehicle] if [parking] is stopped.
  static void _checkAndReactIfVehicleParkedIn(
      Vehicle vehicle, bool parking, Timer timer, BuildContext context) {
    if (!parking) {
      timer.cancel();
      _setParkedIn(vehicle, context);
    }
  }

  /// Sets the [vehicle] to a parked in state.
  /// Creates notification.
  static void _setParkedIn(Vehicle vehicle, BuildContext context) {
    vehicle.setAndUpdateParkedIn(context, true);
    vehicle.setAndUpdateParkIngIn(context, false);
    vehicle.setAndUpdateParkIngOut(context, false);
    print('vehicle parked in: ' + vehicle.parkedIn.toString());

    Notifications.createNotificationClickable(
        AppLocalizations.of(context).notificationParkInTitle +
            vehicle.name +
            ' ' +
            vehicle.licensePlate,
        AppLocalizations.of(context).notificationParkInBody,
        vehicle.inAppKey, (value) {
      if (!vehicle.parkingOut) {
        Navigator.pushReplacementNamed(
            context, vehicle.inAppKey + Routes.parkOut);
      }
      return null;
    }, true, false);
  }

  /// Assumes that vehicle parked in after [_errorLimit] is reached.
  static void _handleGetPositionErrorForParkInRequest(
      int errorCount, Timer timer, BuildContext context) {
    print('Could not update position ' + errorCount.toString());
    if (errorCount++ > _errorLimit) {
      timer.cancel();
      _setParkedIn(vehicle, context);
    }
  }

  /// Returns parking garage occupied dialog.
  static Future _noParkingSpotsAvailable(BuildContext context) {
    print('no parking spots available');
    return showDialog(
        context: context,
        builder: (context) {
          return ParkingGarageOccupiedDialog.getDialog(context);
        });
  }

  /// Sends the [vehicle] park out request to the [ApiProvider].
  static void parkOutRequest(BuildContext context, Vehicle vehicle) {
    if (needsToParkOut(context, vehicle)) {
      // Cancelling park in if needed.
      vehicle.setAndUpdateParkIngIn(context, false);

      vehicle.setAndUpdateParkIngOut(context, true);
      print(vehicle.name + ' parking out');

      int _errorCount = 0;
      // Try to contact server.
      ApiProvider.parkOut(vehicle).then((value) {
        // Update the position while the vehicle is parking out.
        new Timer.periodic(Duration(seconds: 1), (timer) {
          print('ParkOut: updating vehicle ' + vehicle.name);
          ParkManager.updatePosition(vehicle).then((bool parking) {
            print('parking? ' + parking.toString());
            _checkAndReactIfParkOutFailed(vehicle, timer, context);
            _checkAndReactIfVehicleParkedOut(vehicle, parking, timer, context);
          }).catchError((e) {
            _handleGetPositionErrorForParkOutRequest(
                _errorCount, timer, context);
            return;
          });
        });
        // If park out didn't work: NoConnectionDialog and set vehicle state.
      }).catchError((e) => showDialog(
          context: context,
          builder: (context) {
            return NoConnectionDialog.getDialog(context);
          }).then((value) => vehicle.setAndUpdateParkIngOut(context, false)));
    } else {
      print('vehicle is not parked in');
      // vehicle does not need to be parked out.
      Navigator.pushReplacementNamed(context, vehicle.inAppKey);
    }
  }

  /// Returns if [vehicle] needs to be parked out.
  /// Parking in vehicle can be parked out.
  static bool needsToParkOut(BuildContext context, Vehicle vehicle) {
    return (vehicle.parkedIn || vehicle.parkingIn) && !vehicle.parkingOut;
  }

  /// Cancels [timer] and opens [NoConnectionDialog] if [vehicle] stopped parking out.
  static Future _checkAndReactIfParkOutFailed(
      Vehicle vehicle, Timer timer, BuildContext context) {
    if (!vehicle.parkingOut) {
      timer.cancel();
      // if park out didn't work: connection to server failed.
      showDialog(
          context: context,
          builder: (context) {
            return NoConnectionDialog.getDialog(context);
          }).then((value) => _setParkedIn(vehicle, context));
    }
    return null;
  }

  /// Cancels [timer] and calls _setParkedOut() for [vehicle] if [parking] is stopped.
  static void _checkAndReactIfVehicleParkedOut(
      Vehicle vehicle, bool parking, Timer timer, BuildContext context) {
    if (!parking) {
      timer.cancel();
      _setParkedOut(vehicle, context);
    }
  }

  /// Sets the [vehicle] to a parked out state.
  /// Creates notification and park out finished dialog.
  static void _setParkedOut(Vehicle vehicle, BuildContext context) {
    vehicle.setAndUpdateParkedIn(context, false);
    vehicle.setAndUpdateParkIngIn(context, false);
    vehicle.setAndUpdateParkIngOut(context, false);
    print('vehicle parked out: ' + vehicle.parkedIn.toString());

    Navigator.pushReplacementNamed(context, vehicle.inAppKey);
    showDialog(
        context: context,
        builder: (context) {
          return ParkDialogs.getParkOutFinishedDialog(context);
        });

    Notifications.createNotification(
        AppLocalizations.of(context).notificationParkOutTitleOne +
            vehicle.name +
            ' ' +
            vehicle.licensePlate +
            AppLocalizations.of(context).notificationParkOutTitleTwo,
        AppLocalizations.of(context).notificationParkOutBody,
        true,
        false);
  }

  /// Assumes that vehicle parked out after [_errorLimit] is reached.
  static void _handleGetPositionErrorForParkOutRequest(
      int errorCount, Timer timer, BuildContext context) {
    print('Could not update position ' + errorCount.toString());
    if (errorCount++ > _errorLimit) {
      timer.cancel();
      _setParkedOut(vehicle, context);
    }
  }

  /// Widget showing the parking garages map and an overlay for
  /// the vehicle
  /// and the destination
  /// both of them are optional and if neither are specified an empty map will be returned.
  static Widget getParkAnimation(
      {@required BuildContext context,
      Coordinate vehiclePosition,
      destination}) {
    print('generate parkInMap');
    //TODO height must be calculated from aspect ratio of map
    final double _width = MediaQuery.of(context).size.width;
    final double _height = (1473 * _width) / 1000;

    //assume 0x0 to be the bottom left
    Coordinate _topRightAdjusted = Coordinate(
        longitude: currentParkingGarage.topRight.longitude -
            currentParkingGarage.bottomLeft.longitude,
        latitude: currentParkingGarage.topRight.latitude -
            currentParkingGarage.bottomLeft.latitude);

    double _iconSize = 16;
    Container _vehicleIcon = Container(
        width: _iconSize, height: _iconSize, child: Icon(Icons.circle));
    Container _destinationIcon = Container(
        width: _iconSize,
        height: _iconSize,
        child: Icon(
          Icons.circle,
          color: Theme.of(context).accentColor,
        ));

    return Stack(alignment: Alignment.center, children: [
      //map
      Container(
        width: _width,
        height: _height,
        child: Image(
          image: AssetImage(currentParkingGarage.map),
          fit: BoxFit.fill,
        ),
      ),
      //vehicleIcon
      _getPositionedIcon(vehiclePosition, _height, _topRightAdjusted, _iconSize,
          _width, _vehicleIcon),
      //vehicle destination
      _getPositionedIcon(destination, _height, _topRightAdjusted, _iconSize,
          _width, _destinationIcon)
    ]);
  }

  static Positioned _getPositionedIcon(
      Coordinate iconPosition,
      double containerHeight,
      Coordinate topRight,
      double _iconSize,
      double containerWidth,
      Container _icon) {
    Coordinate _iconPositionAdjusted;
    double iconOffsetHeight, iconOffsetWidth;
    Positioned _positionedIcon;
    if (iconPosition != null) {
      _iconPositionAdjusted = Coordinate(
          latitude:
              iconPosition.latitude - currentParkingGarage.bottomLeft.latitude,
          longitude: iconPosition.longitude -
              currentParkingGarage.bottomLeft.longitude);

      //scale the icons position
      iconOffsetHeight = (containerHeight / topRight.latitude) *
              _iconPositionAdjusted.latitude -
          (_iconSize / 2);
      iconOffsetWidth = (containerWidth / topRight.longitude) *
              _iconPositionAdjusted.longitude -
          (_iconSize / 2);

      //icon
      _positionedIcon = Positioned(
        left: iconOffsetWidth,
        bottom: iconOffsetHeight,
        child: _icon,
      );
    }

    //empty positioned
    if (_positionedIcon == null)
      _positionedIcon = Positioned(
        child: Container(),
      );
    return _positionedIcon;
  }

  ///update the position in the vehicle
  ///returns true if the vehicle is still parking
  static Future<bool> updatePosition(Vehicle vehicle) async {
    var position = await ApiProvider.getPosition(vehicle);
    double latitude = position["latitude"];
    double longitude = position["longitude"];
    vehicle.setLocation(latitude, longitude);
    return position["parking"] && !position["reached_position"];
  }
}
