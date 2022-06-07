/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 SIGNET Lab, Department of Information Engineering,
 * University of Padova
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef TERRAIN_LOS_H
#define TERRAIN_LOS_H

#include <string>
#include <iostream>
#include <set>
#include <map>
#include <vector>
#include "ns3/channel-condition-model.h"


namespace ns3 {

class MobilityModel;

/**
 * \ingroup propagation
 *
 * \brief Computes the channel condition according to realistic terrain
 *
 * ADD MORE DETAILED EXPLANATION HERE
 * ALSO ADD PAPER CITE
 */

class TerrainLOS: public ChannelConditionModel
{
  public: 
    
    int VPRow;
    int VPCol;
    
    /**
     * Get the type ID.
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId (void);

    /**
     * Constructor for the TerrainLOS class
     */
    TerrainLOS (int vpRow, int vpCol);
    
    /**
     * Destructor for the TerrainLOS class
     */
    //~TerrainLOS ();
    // Good to make destructors virtual -> force a definition
    virtual ~TerrainLOS ();
    
    void dump (void);
    void hello (void);

    /**
     * Computes the condition of the channel between a and b, that will be always non-LoS
     *
     * \param a mobility model
     * \param b mobility model
     * \return the condition of the channel between a and b, that will be always non-LoS
     */
    // It was virtual in ChannelConditionModel
    virtual Ptr<ChannelCondition> GetChannelCondition (Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const override;

    /**
    * \brief Copy constructor
    *
    * Defined and unimplemented to avoid misuse
    */
    TerrainLOS (const TerrainLOS&) = delete;


    /**
    * \brief Copy constructor
    *
    * Defined and unimplemented to avoid misuse
    * \returns a copy of the object
    */
    TerrainLOS &operator = (const TerrainLOS&) = delete;

    // It was virtual in ChannelConditionModel
    virtual int64_t AssignStreams (int64_t stream) override;
    //virtual int64_t AssignStreams (int64_t stream);

    
    void bootStrap(std::vector<std::vector<float>> dem);
    void refreshAllGrids(std::vector<std::vector<float>> dem);
    
    void refreshAG(std::vector<std::vector<float>> dem);
    void refreshVS(void);
    void refreshVV(void);
    void refreshTG(void);

    void printAuxGrid(void);
    void printVizViews(void);
    void printVizScore(void);
    void printTrackerGrid(void);
    
    void processAllEdges(int vpRow, int vpCol);
    void processAllSlices(int vpRow, int vpCol);  
    

  protected:
    //Ptr<UniformRandomVariable> m_uniformVar; //!< uniform random variable


  private:
    std::string hello_msg = "Hello I am terrainlos\n";

    /**
    * This method computes the channel condition based on a probabilistic model 
    * that is specific for the scenario of interest
    *
    * \param a tx mobility model
    * \param b rx mobility model
    * \return the channel condition
    */
    Ptr<ChannelCondition> ComputeChannelCondition (Ptr<const MobilityModel> a, Ptr<const MobilityModel> b) const;
     
    /**
     * Struct to store the channel condition in the m_channelConditionMap
     */
    struct Item
    {
      Ptr<ChannelCondition> m_condition; //!< the channel condition
      Time m_generatedTime; //!< the time when the condition was generated
    };
    
    std::unordered_map<uint32_t, Item> m_channelConditionMap; //!< map to store the channel conditions
    
    Time m_updatePeriod; //!< the update period for the channel condition
   
    //=============
    struct XY 
    {
      int X;
      int Y;
    };
    
    struct XYZ 
    {
      float X;
      float Y;
      float Z;
    };

    struct IncInfo2
    {
      int rowInc1;
      int colInc1;
    };

    struct IncInfo4 
    {
      int rowInc1;
      int colInc1;
      int rowInc2;
      int colInc2;
    };

    enum class GEdge 
    {
      N, NE, E, SE, S, SW, W, NW
    };

    enum class GSlice 
    {
      NW_N, N_NE, NE_E, E_SE, SE_S, S_SW, SW_W, W_NW
    };
    
    int MaxRow;
    int MaxCol;
 
    std::vector<std::vector<float>> AuxGrid;
    std::vector<std::vector<int>> vizScore;
    std::vector<std::vector<int>> vizViews;
    std::vector<std::vector<int>> TrackerGrid;

    void printXYZ(XYZ p);
    XYZ vecAdd(XYZ p1, XYZ p2);                // p1 + p2
    XYZ vecSub(XYZ p1, XYZ p2);                // p2 - p2
    float vecMag(XYZ p1);                      // |p1|
    float dotProduct(XYZ p1, XYZ p2);          // p1 dot p2
    XYZ scalarMult(float c, XYZ p1);           // c * p1
    float vecAngle(XYZ p1, XYZ p2);            // angle between p1 and p2
    float getD(XYZ a, XYZ b, XYZ c, XYZ d);    // Used for dist between 2 3D lines
    XYZ planeNormal(XYZ p1, XYZ p2, XYZ p3);   // Gets normal vector of a plane
    
    // intersection pt of line and plane
    XYZ linePlaneIntersect(XYZ l1, XYZ l2, XYZ p1, XYZ p2, XYZ p3); 
    
    // Gets min visible height when processing edges
    float minVisHeight(XYZ p1, XYZ p2, XYZ p3, XYZ p4); 
    bool inGrid(int cRow, int cCol);
    std::vector<XY> getSliceIndices(GSlice slc, int iRow, int iCol);
    void processEdge(GEdge se, int vpRow, int vpCol);
    void processSlice(GSlice slc, int vpRow, int vpCol);

    const std::vector<GEdge> allEdges = {GEdge::N, GEdge::NE, GEdge::E, GEdge::SE, 
                                         GEdge::S, GEdge::SW, GEdge::W, GEdge::NW};

    const std::vector<GSlice> allSlices = {GSlice::NW_N, GSlice::N_NE, 
                                           GSlice::NE_E, GSlice::E_SE, 
                                           GSlice::SE_S, GSlice::S_SW, 
                                           GSlice::SW_W, GSlice::W_NW};

    const std::map<GEdge, IncInfo2> mapIncEdge = {{GEdge::N, {-1, 0}},
                                                 {GEdge::NE, {-1, 1}},
                                                 {GEdge::E, {0, 1}},
                                                 {GEdge::SE, {1, 1}},
                                                 {GEdge::S, {1, 0}},
                                                 {GEdge::SW, {1, -1}},
                                                 {GEdge::W, {0, -1}},
                                                 {GEdge::NW, {-1, -1}}};
    
    const std::map<GSlice, IncInfo4> mapIncSlice = {{GSlice::NW_N, {1, 1, 1, 0}},
                                                   {GSlice::N_NE, {1, 0, 1, -1}},
                                                   {GSlice::NE_E, {1, -1, 0, -1}},
                                                   {GSlice::E_SE, {0, -1, -1, -1}},
                                                   {GSlice::SE_S, {-1, -1, -1, 0}},
                                                   {GSlice::S_SW, {-1, 0, -1, 1}},
                                                   {GSlice::SW_W, {-1, 1, 0, 1}},
                                                   {GSlice::W_NW, {0, 1, 1, 1}}};


}; // class TerrainLOS

} // namespace ns3

#endif // TERRAIN_LOS_H





