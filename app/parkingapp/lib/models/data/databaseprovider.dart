import 'package:parkingapp/models/classes/vehicle.dart';
import 'package:sqflite/sqflite.dart';
import 'dart:async';
import 'package:path/path.dart';

class DatabaseProvider {
  static const String TABLE_VEHICLE = "vehicle";
  static const String COLUMN_ID = "id";
  static const String COLUMN_KEY = "key";
  static const String COLUMN_NAME = "name";
  static const String COLUMN_LICENSEPLATE = "licenseplate";
  static const String COLUMN_WIDTH = "width";
  static const String COLUMN_HEIGHT = "height";
  static const String COLUMN_DEPTH = "depth";
  static const String COLUMN_TURNANGLE = "turnangle";
  static const String COLUMN_NEAREXIT = "nearexit";
  static const String COLUMN_PARKINGCARD = "parkingcard";
  static const String COLUMN_DOCHARGE = "docharge";
  static const String COLUMN_CHARGINGPROVIDER = "chargingprovider";
  static const String COLUMN_CHARGEBEGIN = "chargebegin";
  static const String COLUMN_CHARGEEND = "chargeend";
  static const String COLUMN_CHARGE = "charge";

  DatabaseProvider._();
  static final _instance = DatabaseProvider._();
  static DatabaseProvider get = _instance;

  bool isInitialized = false;
  Database _database;

  Future<Database> database() async {
    if (!isInitialized) await _init();
    return _database;
    /*print("database getter called");

    if (_database != null) {
      print("1");
      return _database;
    }
    print("2");
    _database = await createDatabase();
    print("6" + database.toString());

    return _database;*/
  }

  Future _init() async {
    var databasesPath = await getDatabasesPath();
    String path = join(databasesPath, 'vehicleDB.db');

    await openDatabase(path, version: 1,
        onCreate: (Database database, int version) async {
      print("Creating vehicle table");
      await database.execute(
        "CREATE TABLE $TABLE_VEHICLE ("
        "$COLUMN_ID INTEGER PRIMARY KEY,"
        "$COLUMN_KEY TEXT,"
        "$COLUMN_NAME TEXT,"
        "$COLUMN_LICENSEPLATE TEXT,"
        "$COLUMN_WIDTH DOUBLE,"
        "$COLUMN_HEIGHT DOUBLE,"
        "$COLUMN_DEPTH DOUBLE,"
        "$COLUMN_TURNANGLE DOUBLE,"
        "$COLUMN_NEAREXIT INTEGER,"
        "$COLUMN_PARKINGCARD INTEGER,"
        "$COLUMN_DOCHARGE INTEGER,"
        "$COLUMN_CHARGINGPROVIDER TEXT,"
        "$COLUMN_CHARGEBEGIN TEXT,"
        "$COLUMN_CHARGEEND TEXT,"
        "$COLUMN_CHARGE TEXT"
        ")",
      );
    }).then((value) {
      _database = value;
      isInitialized = true;
    });
  }

  Future<Database> createDatabase() async {
    var path = await getDatabasesPath();
    print("3");
    String dbPath = join(path, 'vehicleDB.db');
    print("4");
    Database database = await openDatabase(
      dbPath,
      version: 1,
      onCreate: (Database database, int version) async {
        print("Creating vehicle table");
        await database.execute(
          "CREATE TABLE $TABLE_VEHICLE ("
          "$COLUMN_ID INTEGER PRIMARY KEY,"
          "$COLUMN_KEY TEXT,"
          "$COLUMN_NAME TEXT,"
          "$COLUMN_LICENSEPLATE TEXT,"
          "$COLUMN_WIDTH DOUBLE,"
          "$COLUMN_HEIGHT DOUBLE,"
          "$COLUMN_DEPTH DOUBLE,"
          "$COLUMN_TURNANGLE DOUBLE,"
          "$COLUMN_NEAREXIT INTEGER,"
          "$COLUMN_PARKINGCARD INTEGER,"
          "$COLUMN_DOCHARGE INTEGER,"
          "$COLUMN_CHARGINGPROVIDER TEXT,"
          "$COLUMN_CHARGEBEGIN TEXT,"
          "$COLUMN_CHARGEEND TEXT,"
          "$COLUMN_CHARGE TEXT"
          ")",
        );
      },
    );
    print("5" + database.toString());
    return database;
  }

  Future<List<Vehicle>> getVehicles() async {
    final db = await get.database();

    var vehicles = await db.query(TABLE_VEHICLE, columns: [
      COLUMN_ID,
      COLUMN_KEY,
      COLUMN_NAME,
      COLUMN_LICENSEPLATE,
      COLUMN_WIDTH,
      COLUMN_HEIGHT,
      COLUMN_DEPTH,
      COLUMN_TURNANGLE,
      COLUMN_NEAREXIT,
      COLUMN_PARKINGCARD,
      COLUMN_DOCHARGE,
      COLUMN_CHARGINGPROVIDER,
      COLUMN_CHARGEBEGIN,
      COLUMN_CHARGEEND,
      COLUMN_CHARGE
    ]);

    List<Vehicle> vehicleList = List<Vehicle>();

    vehicles.forEach((currentVehicle) {
      Vehicle vehicle = Vehicle.fromMap(currentVehicle);
      vehicleList.add(vehicle);
    });

    return vehicleList;
  }

  Future<Vehicle> insert(Vehicle vehicle) async {
    final db = await get.database();
    vehicle.id = await db.insert(TABLE_VEHICLE, vehicle.toMap());
    return vehicle;
  }

  Future<int> delete(int id) async {
    final db = await get.database();
    return await db.delete(TABLE_VEHICLE, where: "id = ?", whereArgs: [id]);
  }

  Future<int> update(Vehicle vehicle) async {
    final db = await get.database();
    return await db.update(TABLE_VEHICLE, vehicle.toMap(),
        where: "id = ?", whereArgs: [vehicle.id]);
  }
}
