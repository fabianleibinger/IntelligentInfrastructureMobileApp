import 'dart:async';

import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/resources/apiprovider.dart';
import 'package:parkingapp/dialogs/noconnectiondialog.dart';
import 'package:parkingapp/models/classes/coordinate.dart';
import 'package:parkingapp/notifications/notifications.dart';
import 'package:parkingapp/dialogs/parkdialogs.dart';
import 'package:parkingapp/dialogs/parkinggarageoccupieddialog.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:parkingapp/ui/mainpage/mainpage.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

//how often to retry the park in
int _errorLimit = 20;

/// Utility class that communicates with the [ApiProvider]
/// to manage park processes for vehicles.
class ParkManager {
  /// Sends the [vehicle] park in request to the [ApiProvider].
  static void parkInRequest(BuildContext context, Vehicle vehicle) {
    if (needsToParkIn(context, vehicle)) {
      if (currentParkingGarage.vehicleSpecificSpotsAvailable(vehicle)) {
        int _errorCount = 0;
        vehicle.setAndUpdateParkIngIn(context, true);
        print(vehicle.name + ' parking in');

        // Try to contact server.
        ApiProvider.parkIn(vehicle).then((value) {
          //set the parking Spot for the vehicle
          double latitude = value["latitude"];
          double longitude = value["longitude"];
          Coordinate _destination =
              Coordinate(lattitude: latitude, longitude: longitude);
          vehicle.setParkingSpot(_destination);

          //update the position while the vehicle is parking and check if it has been parked
          new Timer.periodic(Duration(seconds: 1), (timer) {
            print('ParkIn: updating vehicle ' + vehicle.name);
            ParkManager.updatePosition(vehicle).then((bool parking) {
              //vehicle is now parked in
              if (!parking) {
                timer.cancel();
                _setParkedIn(vehicle, context);
              }
            }).catchError((e) {
              print('Could not update position ' + _errorCount.toString());
              //if get position fails try again or cancel park in
              if (_errorCount++ > _errorLimit) {
                timer.cancel();
                _setParkedIn(vehicle, context);
              }
              return;
            });
            _checkAndReactParkInWorked(context, vehicle);
          });
          // vehicle not parking in anymore.
        });
      } else {
        // No parking spots available.
        print('no parking spots available');
        showDialog(
            context: context,
            builder: (context) {
              return ParkingGarageOccupiedDialog.getDialog(context);
            });
      }
    } else {
      // vehicle is already parked in.
      print('vehicle ' + vehicle.name + ' is already parked in');
    }
  }

  ///set the [vehicle] to a parked in state
  static void _setParkedIn(Vehicle vehicle, BuildContext context) {
    vehicle.setAndUpdateParkedIn(context, true);
    vehicle.setAndUpdateParkIngIn(context, false);
    print('vehicle parked in: ' + vehicle.parkedIn.toString());
  }

  /// Returns if [vehicle] needs to be parked in.
  /// Parking out vehicle can't be parked in.
  static bool needsToParkIn(BuildContext context, Vehicle vehicle) {
    ApiProvider.getPosition(vehicle).then((value) {
      value["parkedIn"] != null ??
          vehicle.setAndUpdateParkedIn(context, value["parkedIn"]);
      value["reached_position"] != null ??
          vehicle.setAndUpdateParkIngIn(context, value["reached_position"]);
      print('needs to park in? parked in: ' +
          vehicle.parkedIn.toString() +
          ' parking in: ' +
          vehicle.parkingIn.toString());
    });
    return !vehicle.parkedIn && !vehicle.parkingIn && !vehicle.parkingOut;
  }

  /// Checks if park in worked,
  /// creates parked in [Notification] or opens [NoConnectionDialog].
  static void _checkAndReactParkInWorked(
      BuildContext context, Vehicle vehicle) {
    if (vehicle.parkedIn) {
      // if park in worked: notification, that triggers parkOut method.
      Notifications.createNotificationClickable(
          AppLocalizations.of(context).notificationParkInTitle +
              vehicle.name +
              ' ' +
              vehicle.licensePlate,
          AppLocalizations.of(context).notificationParkInBody,
          vehicle.inAppKey, (value) {
        vehicle.parkOut(context);
        return null;
      }, true, false);
    } else if (!vehicle.parkedIn && !vehicle.parkingIn) {
      // vehicle is not parked in but also not parking in meaning something has failed
      // if park in didn't work: connection to server failed.
      showDialog(
          context: context,
          builder: (context) {
            return NoConnectionDialog.getDialog(context);
          });
    }
  }

  /// Sends the [vehicle] park out request to the [ApiProvider].
  static void parkOutRequest(BuildContext context, Vehicle vehicle) {
    if (needsToParkOut(context, vehicle)) {
      int _errorCount = 0;
      // Cancelling park in if needed.
      vehicle.setAndUpdateParkIngIn(context, false);

      vehicle.setAndUpdateParkIngOut(context, true);
      print(vehicle.name + ' parking out');

      // Try to contact server.
      ApiProvider.parkOut(vehicle).then((value) {
        //update the position while the vehicle is parking out
        new Timer.periodic(Duration(seconds: 1), (timer) {
          print('ParkOut: updating vehicle ' + vehicle.name);
          ParkManager.updatePosition(vehicle).then((bool parking) {
            print('parking? ' + parking.toString());
            //park out finished
            if (!parking) {
              timer.cancel();
              vehicle.setAndUpdateParkedIn(context, false);
              vehicle.setAndUpdateParkIngOut(context, false);
              print('vehicle parked out: ' + vehicle.parkedIn.toString());
              // Open main page.
              Navigator.pushReplacementNamed(context, vehicle.inAppKey);
              showDialog(
                  context: context,
                  builder: (context) {
                    return ParkDialogs.getParkOutFinishedDialog(context);
                  });
            }
          }).catchError((e) {
            print('Could not update position ' + _errorCount.toString());
            //if get position fails try again or cancel park in
            if (_errorCount++ > _errorLimit) {
              timer.cancel();
              vehicle.setAndUpdateParkedIn(context, false);
              vehicle.setAndUpdateParkIngOut(context, false);
              print('vehicle parked out: ' + vehicle.parkedIn.toString());
              // Open main page.
              Navigator.pushReplacementNamed(context, vehicle.inAppKey);
              showDialog(
                  context: context,
                  builder: (context) {
                    return ParkDialogs.getParkOutFinishedDialog(context);
                  });
            }
            return;
          });
          _checkAndReactParkOutWorked(context, vehicle);
        });

        // vehicle not parking out anymore.
      });
    } //else {
    //print('vehicle is not parked in');
    //vehicle does not need to be parked out
    //Navigator.pushReplacementNamed(context, vehicle.inAppKey);
    //}
  }

  /// Returns if [vehicle] needs to be parked out.
  /// Parking in vehicle can be parked out.
  static bool needsToParkOut(BuildContext context, Vehicle vehicle) {
    ApiProvider.getPosition(vehicle).then((value) {
      print(value);
      value["reached_position"] != null ??
          vehicle.setAndUpdateParkedIn(context, value["reached_position"]);
      value["parking"] != null ??
          vehicle.setAndUpdateParkIngOut(context, value["parking"]);
    });
    return vehicle.parkedIn;
  }

  /// Checks if park out worked,
  /// creates parked out [Notification] or opens [NoConnectionDialog].
  static void _checkAndReactParkOutWorked(
      BuildContext context, Vehicle vehicle) {
    if (!vehicle.parkedIn) {
      // if park out worked: notification
      Notifications.createNotification(
          AppLocalizations.of(context).notificationParkOutTitleOne +
              vehicle.name +
              ' ' +
              vehicle.licensePlate +
              AppLocalizations.of(context).notificationParkOutTitleTwo,
          AppLocalizations.of(context).notificationParkOutBody,
          true,
          false);
    } else if (vehicle.parkedIn && !vehicle.parkingOut) {
      // if park out didn't work: connection to server failed
      showDialog(
          context: context,
          builder: (context) {
            return NoConnectionDialog.getDialog(context);
          });
    }
  }

  ///Widget showing the parking garages map and an overlay for
  ///the vehicle
  ///and the destination
  ///both of them are optional and if neither are specified an empty map will be returned
  static Widget getParkInAnimation(
      {@required BuildContext context,
      Coordinate vehiclePosition,
      destination}) {
    print('generate parkInMap');
    //TODO height must be calculated from aspect ratio of mapp
    final double _width = MediaQuery.of(context).size.width;
    final double _height = (1473 * _width) / 1000;

    //assume 0x0 to be the bottom left
    Coordinate _topRightAdjusted = Coordinate(
        longitude: currentParkingGarage.topRight.longitude -
            currentParkingGarage.bottomLeft.longitude,
        lattitude: currentParkingGarage.topRight.lattitude -
            currentParkingGarage.bottomLeft.lattitude);

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
          lattitude: iconPosition.lattitude -
              currentParkingGarage.bottomLeft.lattitude,
          longitude: iconPosition.longitude -
              currentParkingGarage.bottomLeft.longitude);

      //scale the icons position
      iconOffsetHeight = (containerHeight / topRight.lattitude) *
              _iconPositionAdjusted.lattitude -
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
    vehicle.setLocation(Coordinate(lattitude: latitude, longitude: longitude));
    return position["parking"] && !position["reached_position"];
  }
}
