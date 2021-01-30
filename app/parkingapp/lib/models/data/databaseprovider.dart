import 'package:parkingapp/models/classes/loadablevehicle.dart';
import 'package:sqflite/sqflite.dart';
import 'dart:async';
import 'package:path/path.dart';

class DatabaseProvider {
  static const String TABLE_VEHICLE = "vehicle";
  static const String COLUMN_ID = "id";
  static const String COLUMN_KEY = "key";
  static const String COLUMN_NAME = "name";
  static const String COLUMN_LICENSE_PLATE = "licensePlate";
  static const String COLUMN_WIDTH = "width";
  static const String COLUMN_HEIGHT = "height";
  static const String COLUMN_LENGTH = "length";
  static const String COLUMN_TURNING_CYCLE = "turningCycle";
  static const String COLUMN_NEAR_EXIT_PREFERENCE = "nearExitPreference";
  static const String COLUMN_PARKING_CARD = "parkingCard";
  static const String COLUMN_DO_CHARGE = "doCharge";
  static const String COLUMN_CHARGING_PROVIDER = "chargingProvider";
  static const String COLUMN_CHARGE_TIME_BEGIN = "chargeTimeBegin";
  static const String COLUMN_CHARGE_TIME_END = "chargeTimeEnd";
  static const String COLUMN_CHARGE = "charge";

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
    String dbPath = await getDatabasesPath();

    return await openDatabase(
      join(dbPath, 'vehicleDB.db'),
      version: 1,
      onCreate: (Database database, int version) async {
        print("Creating vehicle table");

        await database.execute(
          "CREATE TABLE $TABLE_VEHICLE ("
          "$COLUMN_ID INTEGER PRIMARY KEY,"
          "$COLUMN_KEY TEXT,"
          "$COLUMN_NAME TEXT,"
          "$COLUMN_LICENSE_PLATE TEXT,"
          "$COLUMN_WIDTH DOUBLE,"
          "$COLUMN_HEIGHT DOUBLE,"
          "$COLUMN_LENGTH DOUBLE,"
          "$COLUMN_TURNING_CYCLE DOUBLE,"
          "$COLUMN_NEAR_EXIT_PREFERENCE INTEGER,"
          "$COLUMN_PARKING_CARD INTEGER,"
          "$COLUMN_DO_CHARGE INTEGER,"
          "$COLUMN_CHARGING_PROVIDER TEXT,"
          "$COLUMN_CHARGE_TIME_BEGIN TEXT,"
          "$COLUMN_CHARGE_TIME_END TEXT,"
          "$COLUMN_NEAR_EXIT_PREFERENCE TEXT"
          ")",
        );
      },
    );
  }

  Future<List<LoadableVehicle>> getVehicles() async {
    final db = await database;

    var vehicles = await db.query(TABLE_VEHICLE, columns: [
      COLUMN_ID,
      COLUMN_KEY,
      COLUMN_NAME,
      COLUMN_LICENSE_PLATE,
      COLUMN_WIDTH,
      COLUMN_HEIGHT,
      COLUMN_LENGTH,
      COLUMN_TURNING_CYCLE,
      COLUMN_NEAR_EXIT_PREFERENCE,
      COLUMN_PARKING_CARD,
      COLUMN_DO_CHARGE,
      COLUMN_CHARGING_PROVIDER,
      COLUMN_CHARGE_TIME_BEGIN,
      COLUMN_CHARGE_TIME_END,
      COLUMN_CHARGE
    ]);

    List<LoadableVehicle> vehicleList = <LoadableVehicle>[];

    vehicles.forEach((currentVehicle) {
      LoadableVehicle vehicle = LoadableVehicle.fromMap(currentVehicle);
      vehicleList.add(vehicle);
    });

    return vehicleList;
  }

  Future<LoadableVehicle> insert(LoadableVehicle vehicle) async {
    final db = await database;
    vehicle.id = await db.insert(TABLE_VEHICLE, vehicle.toMap());
    return vehicle;
  }

  Future<int> delete(int id) async {
    final db = await database;
    return await db.delete(TABLE_VEHICLE, where: "id = ?", whereArgs: [id]);
  }

  Future<int> update(LoadableVehicle vehicle) async {
    final db = await database;
    return await db.update(TABLE_VEHICLE, vehicle.toMap(),
        where: "id = ?", whereArgs: [vehicle.id]);
  }
}
