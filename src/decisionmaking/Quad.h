#ifndef ELIKOS_DUNGEONMASTER_QUAD_H
#define ELIKOS_DUNGEONMASTER_QUAD_H

/**
 * \file Quad.h
 * \brief Quad class declaration
 * \author christophebedard
 */

#include <geometry_msgs/Pose.h>

/**
 * \class Quad
 * \brief class which encapsulates relevant info for the quad.
 */
class Quad
{
public:
    /**
     * \brief Constructor.
     * 
     * \param pose : the pose of the quad.
     */
    Quad(geometry_msgs::Pose pose);

    /**
     * \brief Destructor.
     */
    ~Quad();

    /**
     * \brief Position accessor.
     * 
     * \return the latest position.
     */
    geometry_msgs::Pose getPose() const;

    /**
     * \brief Update quad pose.
     * 
     * \param pose : the latest pose.
     */
    void updatePose(geometry_msgs::Pose pose);

private:
    geometry_msgs::Pose pose_; /**< the position of the target robot */

    /**
     * \brief Deleted default constructor.
     */
    Quad() = delete;
};

#endif // ELIKOS_DUNGEONMASTER_QUAD_H