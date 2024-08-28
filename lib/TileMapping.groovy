import mil.nga.crs.CRS
import mil.nga.crs.util.proj.ProjParser
import mil.nga.crs.wkt.CRSReader
import org.locationtech.jts.geom.*
import org.locationtech.proj4j.CRSFactory
import org.locationtech.proj4j.CoordinateReferenceSystem
import org.locationtech.proj4j.ProjCoordinate
import org.locationtech.proj4j.CoordinateTransform
import org.locationtech.proj4j.CoordinateTransformFactory

import java.io.BufferedReader
import java.io.FileReader
import java.io.IOException
import java.util.*
import java.util.stream.Collectors
import java.nio.file.Path

class TileMapping {

    final CoordinateReferenceSystem standardCRS;
    private CoordinateReferenceSystem targetCrs;
    double projectedXOrigin;
    double projectedYOrigin;
    double tileSize;

    final CoordinateTransform crsTransfomer;

    LandsatMetadataParser landsatMetadataParser;

    // stores references to force tile polygon objects to avoid recreating them
    Map<int [], Polygon> tileCache;

    // reusable factories
    CRSFactory crsFactory;
    GeometryFactory geometryFactory;


    /**
     * @param pathToDatacube: path to force datacube file, usually named datacube-definition.prj
     */
    TileMapping(String pathToDatacube){

        crsFactory = new CRSFactory();
        geometryFactory = new GeometryFactory();
        tileCache = new HashMap<>();
        landsatMetadataParser = new LandsatMetadataParser();

        try {
            BufferedReader reader = new BufferedReader(new FileReader(pathToDatacube));

            // read necessary CRS (coordinate reference system) information from datacube
            String crsInfo = reader.readLine();
            reader.readLine();
            reader.readLine();
            this.projectedXOrigin = Double.parseDouble(reader.readLine());
            this.projectedYOrigin = Double.parseDouble(reader.readLine());
            this.tileSize = Double.parseDouble(reader.readLine());
            reader.close();

            // get projection
            CRS crs = CRSReader.read(crsInfo);
            String projText = ProjParser.paramsText(crs);
            this.targetCrs = this.crsFactory.createFromParameters("Datacube_CRS", projText);

        } catch (IOException e) {
            e.printStackTrace();
            System.exit(1);
        }

        // standard lon/lat CRS
        standardCRS = crsFactory.createFromName("EPSG:4326");


        // create transformer form lon/lat CRS to datacube CRS
        CoordinateTransformFactory transformFactory = new CoordinateTransformFactory();
        this.crsTransfomer = transformFactory.createTransform(this.standardCRS, this.targetCrs);
    }

    /**
     * Find all force tiles that intersect a given landsat scene and
     * create a mapping that assigns the intersection area to the force tile String.
     * @param path2LandsatMetadataFile: path to landsat metadata file for the targeted scene
     * @return Mapping of force tile string (e.g. X0000_Y0000) to intersection area (Double)
     */
    Map<String,Double> getTileMap(Path path2LandsatMetadataFile){

        path2LandsatMetadataFile = path2LandsatMetadataFile.toRealPath()

        // parse landsat metadata file
        landsatMetadataParser.parse(path2LandsatMetadataFile);


        // transform corners to target crs in proj4j format
        ProjCoordinate[] projCoordinates = Arrays.stream(landsatMetadataParser.getCorners())
                .map(corner -> transformCoordinates(corner[0], corner[1]))
                .toArray(ProjCoordinate[]::new);


        // create polygon for Landsat tile
        Coordinate[] jstCoordinates = Arrays.stream(projCoordinates)
                .map(corner -> new Coordinate(corner.x, corner.y))
                .toArray(Coordinate[]::new);

        Polygon landsatPolygon = createPolygon(jstCoordinates[0], jstCoordinates[1], jstCoordinates[2], jstCoordinates[3]);


        // get min and max force tile rows/columns that may intersect landsat tile
        int[] forceTileRange = Arrays.stream(projCoordinates)
                .map(this::computeTile)
                .reduce(new int[] {Integer.MAX_VALUE, Integer.MIN_VALUE, Integer.MAX_VALUE, Integer.MIN_VALUE},
                        (accumulator, element) -> {
                            accumulator[0] = Math.min(accumulator[0], element[0]); // x min
                            accumulator[1] = Math.max(accumulator[1], element[0]); // x max
                            accumulator[2] = Math.min(accumulator[2], element[1]); // y min
                            accumulator[3] = Math.max(accumulator[3], element[1]); // y max
                            return accumulator;
                        });

        // derive all force tiles that need to be considered for tile mapping
        List<int[]> forceTiles = new ArrayList<>();
        for (int x = forceTileRange[0]; x <= forceTileRange[1]; x++){
            for (int y = forceTileRange[2]; y <= forceTileRange[3]; y++){
                forceTiles.add(new int[] {x,y});
            }
        }

        // compute tile mapping
        Map<String, Double> tileMapping = forceTiles.stream()
                .map(this::getTilePolygon) // get polygons for all force tiles
                .collect(Collectors.toMap( // compute final mapping of force tile->intersection area with landsat tile
                        entry -> getTileString(entry.getKey()),
                        entry -> {
                            Geometry intersection = entry.getValue().intersection(landsatPolygon);
                            if (intersection instanceof Polygon){
//                                System.out.println("Intersection area: " + intersection.getArea());
                                return intersection.getArea();
                            } else {
                                return 0.0;
                            }
                        })
                );

        // remove tiles with no intersection
        tileMapping.entrySet().removeIf(entry -> entry.getValue() == 0);

        return tileMapping;
    }

    /**
     * Transform lon/lat coordinates to datacube CRS
     * @param lon: longitude value
     * @param lat: latitude value
     * @return transformed proj4j coordinate object
     */
    private ProjCoordinate transformCoordinates( double lon, double lat){
        ProjCoordinate srcCoordinate = new ProjCoordinate(lon, lat);
        ProjCoordinate targetCoordinate = new ProjCoordinate();
        return crsTransfomer.transform(srcCoordinate, targetCoordinate);
    }

    /**
     * Create a polygon object from four corner coordinates
     * @param upperLeft: Upper left corner jts Coordinate Object
     * @param upperRight: Upper right corner jts Coordinate Object
     * @param lowerLeft: Lower left corner jts Coordinate Object
     * @param lowerRight: Lower Right corner jts Coordinate Object
     * @return jts Polygon Object
     */
    private Polygon createPolygon(Coordinate upperLeft, Coordinate upperRight, Coordinate lowerLeft, Coordinate lowerRight){
        LinearRing ring = geometryFactory.createLinearRing(new Coordinate[]{upperLeft, upperRight, lowerRight, lowerLeft, upperLeft});
        return geometryFactory.createPolygon(ring);
    }

    /**
     * Compute the force tile that contains a given coordinate
     * @param coordinate proj4j Coordinate Object in datacube CRS
     * @return Array with two integers representing the X and Y force tile row and column
     */
    private int[] computeTile(ProjCoordinate coordinate){
        int xT = (int) Math.floor((coordinate.x - this.projectedXOrigin)/this.tileSize);
        int yT = (int) Math.floor((this.projectedYOrigin - coordinate.y)/this.tileSize);
        return new int[]{xT, yT};
    }

    /**
     * Convert a give Array of two integers to their force tile representation
     * e.g. [0,1] -> "X0000_Y0001"
     * @param tile Array containing force tile row and column as integers
     * @return force tile string
     */
    private String getTileString(int[] tile){
        return "X" + String.format("%0" + 4 + "d", tile[0]) + "_Y" + String.format("%0" + 4 + "d", tile[1]);
    }

    /**
     * Compute or load polygon for a given force tile
     * @param tile_xy force tile row and column in an integer array
     * @return jts Polygon object representing the force tile
     */
    private Map.Entry<int[], Polygon> getTilePolygon(int[] tile_xy){
        if (tileCache.containsKey(tile_xy)){
            return new AbstractMap.SimpleEntry<>(tile_xy, tileCache.get(tile_xy));
        } else {
            double xMin = projectedXOrigin + tile_xy[0] * tileSize ;
            double xMax = xMin + tileSize;
            double yMin = projectedYOrigin - tile_xy[1] * tileSize;
            double yMax = yMin - tileSize;

            Polygon tilePolygon = createPolygon(
                    new Coordinate(xMin, yMin),
                    new Coordinate(xMax, yMin),
                    new Coordinate(xMin, yMax),
                    new Coordinate(xMax, yMax)
            );
            tileCache.put(tile_xy, tilePolygon);
            return new AbstractMap.SimpleEntry<>(tile_xy, tilePolygon);
        }
    }

}


/**
 * class for parsing landsat metadata files
 */
class LandsatMetadataParser {

    private double resolution;
    private double upperLeftLat;
    private double upperLeftLon;
    private double upperRightLat;
    private double upperRightLon;
    private double lowerLeftLat;
    private double lowerLeftLon;
    private double lowerRightLat;
    private double lowerRightLon;

    LandsatMetadataParser(){
    }

    /**
     * Collect all necessary information from a Landsat Metadat file
     * @param path2LandsatMetafile: path to metadata file
     */
    void parse (Path path2LandsatMetafile) {
        try {
            BufferedReader reader = new BufferedReader(new FileReader(path2LandsatMetafile.toFile()));

            boolean foundResolution = false;
            String line;
            while ((line = reader.readLine()) != null) {
                if (line.contains("CORNER_UL_LAT_PRODUCT")) {
                    upperLeftLat = parseAssignment(line);
                }
                else if (line.contains("CORNER_UL_LON_PRODUCT")) {
                    upperLeftLon = parseAssignment(line);
                }
                else if (line.contains("CORNER_UR_LAT_PRODUCT")) {
                    upperRightLat = parseAssignment(line);
                }
                else if (line.contains("CORNER_UR_LON_PRODUCT")) {
                    upperRightLon = parseAssignment(line);
                }
                else if (line.contains("CORNER_LL_LON_PRODUCT")) {
                    lowerLeftLon = parseAssignment(line);
                }
                else if (line.contains("CORNER_LL_LAT_PRODUCT")) {
                    lowerLeftLat = parseAssignment(line);
                }
                else if (line.contains("CORNER_LR_LON_PRODUCT")) {
                    lowerRightLon = parseAssignment(line);
                }
                else if (line.contains("CORNER_LR_LAT_PRODUCT")) {
                    lowerRightLat = parseAssignment(line);
                }
                else if (!foundResolution && line.contains("GRID_CELL_SIZE_REFLECTIVE")) {
                    resolution = parseAssignment(line);
                    foundResolution = true;
                }
            }
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Get landsat product corners
     * @return array of 4 integers arrays representing the upper left, upper right, lower left and lower right corner of the product in lon/lat CRS
     */
    double[][] getCorners(){
        double[][] corners = new double[4][];
        corners[0] = new double[] {upperLeftLon,  upperLeftLat};
        corners[1] = new double[] {upperRightLon, upperRightLat};
        corners[2] = new double[] {lowerLeftLon,  lowerLeftLat};
        corners[3] = new double[] {lowerRightLon, lowerRightLat};
        return corners;
    }

    // helper function for parsing assignments in landsat metadata files
    private double parseAssignment(String line){
        return Double.parseDouble(line.substring(line.indexOf("=") +1));
    }
}