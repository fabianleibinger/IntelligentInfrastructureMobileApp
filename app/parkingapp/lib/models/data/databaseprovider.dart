import 'package:parkingapp/models/classes/chargeablevehicle.dart';
import 'package:parkingapp/models/classes/standardvehicle.dart';
import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:sqflite/sqflite.dart';
import 'dart:async';
import 'package:path/path.dart';

class DatabaseProvider {
  static const String TABLE_VEHICLE = "vehicle";
  static const String COLUMN_DATABASE_ID = "databaseId";
  static const String COLUMN_IN_APP_KEY = "inAppKey";
  static const String COLUMN_NAME = "name";
  static const String COLUMN_LICENSE_PLATE = "licensePlate";
  static const String COLUMN_WIDTH = "width";
  static const String COLUMN_HEIGHT = "height";
  static const String COLUMN_LENGTH = "length";
  static const String COLUMN_TURNING_CYCLE = "turningCycle";
  static const String COLUMN_NEAR_EXIT_PREFERENCE = "nearExitPreference";
  static const String COLUMN_PARKING_CARD = "parkingCard";
  static const String COLUMN_PARKED_IN = "parkedIn";
  static const String COLUMN_DO_CHARGE = "doCharge";
  static const String COLUMN_CHARGING_PROVIDER = "chargingProvider";
  static const String COLUMN_CHARGE_TIME_BEGIN = "chargeTimeBegin";
  static const String COLUMN_CHARGE_TIME_END = "chargeTimeEnd";

  DatabaseProvider._();
  static final DatabaseProvider db = DatabaseProvider._();

  Database _database;

  Future<Database> get database async {
    print("database getter called");

    if (_database != null) {
      return _database;
    }

    _database = await createDatabase();

    return _database;
  }

  Future<Database> createDatabase() async {
    var path = await getDatabasesPath();
    String dbPath = join(path, 'vehicleDB.db');

    return await openDatabase(
      dbPath,
      version: 1,
      onCreate: (Database database, int version) async {
        print("Creating vehicle table");
        await database.execute(
          "CREATE TABLE $TABLE_VEHICLE ("
          "$COLUMN_DATABASE_ID INTEGER PRIMARY KEY,"
          "$COLUMN_IN_APP_KEY TEXT,"
          "$COLUMN_NAME TEXT,"
          "$COLUMN_LICENSE_PLATE TEXT,"
          "$COLUMN_WIDTH DOUBLE,"
          "$COLUMN_HEIGHT DOUBLE,"
          "$COLUMN_LENGTH DOUBLE,"
          "$COLUMN_TURNING_CYCLE DOUBLE,"
          "$COLUMN_NEAR_EXIT_PREFERENCE INTEGER,"
          "$COLUMN_PARKING_CARD INTEGER,"
          "$COLUMN_PARKED_IN INTEGER,"
          "$COLUMN_DO_CHARGE INTEGER,"
          "$COLUMN_CHARGING_PROVIDER TEXT,"
          "$COLUMN_CHARGE_TIME_BEGIN TEXT,"
          "$COLUMN_CHARGE_TIME_END TEXT"
          ")",
        );
      },
    );
  }

  Future<List<Vehicle>> getVehicles() async {
    final db = await database;

    var vehicles = await db.query(TABLE_VEHICLE, columns: [
      COLUMN_DATABASE_ID,
      COLUMN_IN_APP_KEY,
      COLUMN_NAME,
      COLUMN_LICENSE_PLATE,
      COLUMN_WIDTH,
      COLUMN_HEIGHT,
      COLUMN_LENGTH,
      COLUMN_TURNING_CYCLE,
      COLUMN_NEAR_EXIT_PREFERENCE,
      COLUMN_PARKING_CARD,
      COLUMN_PARKED_IN,
      COLUMN_DO_CHARGE,
      COLUMN_CHARGING_PROVIDER,
      COLUMN_CHARGE_TIME_BEGIN,
      COLUMN_CHARGE_TIME_END
    ]);

    List<Vehicle> vehicleList = List<Vehicle>();

    vehicles.forEach((currentVehicle) {
      if (currentVehicle[COLUMN_DO_CHARGE] == null) {
        StandardVehicle vehicle = StandardVehicle.fromMap(currentVehicle);
        vehicleList.add(vehicle);
      } else {
        ChargeableVehicle vehicle = ChargeableVehicle.fromMap(currentVehicle);
        vehicleList.add(vehicle);
      }
    });
    return vehicleList;
  }

  Future<Vehicle> insert(Vehicle vehicle) async {
    final db = await database;
    vehicle.databaseId = await db.insert(TABLE_VEHICLE, vehicle.toMap());
    return vehicle;
  }

  Future<int> delete(int id) async {
    final db = await database;
    return await db.delete(TABLE_VEHICLE,
        where: "$COLUMN_DATABASE_ID = ?", whereArgs: [id]);
  }

  Future<int> update(Vehicle vehicle) async {
    print('DB update: inAppKey: ' +
        vehicle.inAppKey +
        ' name: ' +
        vehicle.name +
        ' licensePlate: ' +
        vehicle.licensePlate +
        ' width: ' +
        vehicle.width.toString() +
        ' height: ' +
        vehicle.height.toString() +
        ' length: ' +
        vehicle.length.toString() +
        ' turningCycle: ' +
        vehicle.turningCycle.toString() +
        ' nearExitPreference: ' +
        vehicle.nearExitPreference.toString() +
        ' parkingCard: ' +
        vehicle.parkingCard.toString() +
        ' parkedIn: ' +
        vehicle.parkedIn.toString() +
        ' databaseId: ' +
        vehicle.databaseId.toString());
    final db = await database;
    return await db.update(TABLE_VEHICLE, vehicle.toMap(),
        where: "$COLUMN_DATABASE_ID = ?", whereArgs: [vehicle.databaseId]);
  }

  Future clear() async {
    var path = await getDatabasesPath();
    String dbPath = join(path, 'vehicleDB.db');
    _database.delete(TABLE_VEHICLE);
    //await deleteDatabase(dbPath);
  }
}
