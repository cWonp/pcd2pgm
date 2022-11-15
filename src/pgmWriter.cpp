#include "../include/pgmWriter/pgmWriter.hpp"

/**
 * @brief Loading pcd map file
 *
 * @param mapfileName Loading map file name .pcd
 * @return int Return success
 */
int pgmWriter::setPcdMap(std::string mapfileName)
{
    this->pcdMap.reset(new pointCloud());
    if (pcl::io::loadPCDFile<point>(mapfileName, *(this->pcdMap)) == -1)
    {
        return -1; // Fail to read pcd map file
    }
    return 0; // Success to read pcd map file
}

void pgmWriter::zAxisConditionalFilter(pointCloudPtr inputCloud,
                                       double zMin, double zMax,
                                       pointCloudPtr outputCloud)
{
    pcl::ConditionAnd<point>::Ptr rangeCondition(new pcl::ConditionAnd<point>());
    rangeCondition->addComparison(pcl::FieldComparison<point>::ConstPtr(new pcl::FieldComparison<point>("z", pcl::ComparisonOps::GT, zMin)));
    rangeCondition->addComparison(pcl::FieldComparison<point>::ConstPtr(new pcl::FieldComparison<point>("z", pcl::ComparisonOps::LT, zMax)));

    pcl::ConditionalRemoval<point> condRemoval;
    condRemoval.setCondition(rangeCondition);
    condRemoval.setInputCloud(inputCloud);
    condRemoval.setKeepOrganized(true);
    condRemoval.filter(*outputCloud);
}

int pgmWriter::pcdToPgm(pointCloudPtr pcdPoint, const std::string &pgmFileName)
{
    point minPoint, maxPoint;
    pcl::getMinMax3D(*pcdPoint, minPoint, maxPoint);

    int maxX = abs(minPoint.x) > abs(maxPoint.x) ? abs(minPoint.x) : abs(maxPoint.x);
    int maxY = abs(minPoint.y) > abs(maxPoint.y) ? abs(minPoint.y) : abs(maxPoint.y);

    int mapWidth = ((maxX + margin_m) * 2) * (1 / this->resolution);
    int mapHeight = ((maxY + margin_m) * 2) * (1 / this->resolution);
    //(abs(minPoint.y) + abs(maxPoint.y) + this->margin_m) * 1 / this->resolution;

    std::vector<std::vector<uint8_t>> gridMapValue;

    // Initialize gridMapValue as unknown space
    for (int i = 0; i < mapHeight; i++)
    {
        std::vector<uint8_t> temp;
        temp.resize(mapWidth, unKnownSpace);
        gridMapValue.push_back(temp);
    }

    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*pcdPoint, *pcdPoint, indices);

    // Fill floor as freeSpace
    pointCloudPtr floorPointCloud(new pointCloud);
    this->zAxisConditionalFilter(pcdPoint, floorZmin, floorZmax, floorPointCloud);

    for (const auto &pt : floorPointCloud->points)
    {
        if (std::isnan(pt.x) ||
            std::isnan(pt.y) ||
            std::isnan(pt.z))
        {
            continue;
        }

        int col = (pt.x + (maxX + margin_m)) * (1 / this->resolution);
        int row = (pt.y + (maxY + margin_m)) * (1 / this->resolution);

        gridMapValue[row][col] = freeSpace;
    }

    pcl::octree::OctreePointCloudSearch<point> octree(0.1f);
    octree.setInputCloud(floorPointCloud);
    octree.addPointsFromInputCloud();

    for (int r = 0; r < mapHeight; r++)
    {
        for (int c = 0; c < mapWidth; c++)
        {
            point searchPoint;
            searchPoint.x = (c * this->resolution) - (maxX + margin_m);
            searchPoint.y = (r * this->resolution) - (maxY + margin_m);
            searchPoint.z = 0.0;

            std::vector<int> searchIdx;
            std::vector<float> searchDistance;
            octree.radiusSearch(searchPoint, this->searchRadius, searchIdx, searchDistance);

            if (searchIdx.size() > 0)
                gridMapValue[r][c] = freeSpace;
        }
    }

    pointCloudPtr collisionPointCloud(new pointCloud);
    this->zAxisConditionalFilter(pcdPoint, collisionZmin, collisionZmax, collisionPointCloud);

    for (const auto &pt : collisionPointCloud->points)
    {
        if (std::isnan(pt.x) ||
            std::isnan(pt.y) ||
            std::isnan(pt.z))
        {
            continue;
        }

        int col = (pt.x + (maxX + margin_m)) * (1 / this->resolution);
        int row = (pt.y + (maxY + margin_m)) * (1 / this->resolution);

        gridMapValue[row][col] = collisionSpace;
    }

    std::ofstream outPgmstream;
    outPgmstream.open(pgmFileName + ".pgm");
    outPgmstream << "P2" << '\n';
    outPgmstream << mapWidth << ' ' << mapHeight << '\n';
    outPgmstream << 255 << '\n';

    for (int row = 0; row < mapHeight; row++)
    {
        for (int col = 0; col < mapWidth; col++)
        {
            outPgmstream << (int)gridMapValue[row][col] << ' ';
        }
        outPgmstream << '\n';
    }

    outPgmstream.close();

    return true;
}

void pgmWriter::run()
{
    this->setPcdMap("/home/cocel/workspace/lastmile_cm4_/src/localization/map/TestMapCleanup_SetCentroid.pcd");
    this->pgmFileName = "/home/cocel/workspace/lastmile_cm4_/src/localization/map/TestPGM";
    this->pcdToPgm(this->pcdMap, this->pgmFileName);
}

int main(int argc, char **argv)
{
    pgmWriter writer;
    writer.run();

    return 0;
}