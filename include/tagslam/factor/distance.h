/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/factor/factor.h"
#include "tagslam/tag_factory.h"
#include "tagslam/geometry.h"
#include "tagslam/tag2.h"
#include <ros/ros.h>

namespace tagslam {
  namespace factor {
    class Distance: public Factor {
    public:
      typedef std::shared_ptr<factor::Distance> DistanceFactorPtr;
      typedef std::shared_ptr<const factor::Distance> DistanceFactorConstPtr;
      typedef std::vector<DistanceFactorPtr> DistanceFactorPtrVec;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      Distance(double dist,  double noise,
               const int corn1, const Tag2ConstPtr &tag1,
               const int corn2, const Tag2ConstPtr &tag2,
               const std::string  &name);
      // ------ inherited methods -----
      std::string getLabel() const override;
      VertexId    getId() const override { return (name_);}
      std::shared_ptr<Vertex> clone() const override {
        return (std::shared_ptr<Distance>(new Distance(*this))); }
      VertexDesc addToGraph(const VertexPtr &vp, Graph *g) const override;

      void addToOptimizer(Graph *g) const override;
      bool establishesValues() const override { return (false); }

      // --------- own methods
      double getDistance() const { return (distance_); }
      double getNoise()    const { return (noise_); }
      double distance(const Transform &T_w_b1,
                      const Transform &T_b1_o,
                      const Transform &T_w_b2,
                      const Transform &T_b2_o) const;
      const Tag2ConstPtr getTag(int idx) const { return (tag_[idx]); }
      const Eigen::Vector3d getCorner(int idx) const;
      // --- static methods
      inline static std::shared_ptr<const factor::Distance> cast_const(
        const VertexPtr &vp) {
        return (std::dynamic_pointer_cast<const factor::Distance>(vp));
      }
      inline static std::shared_ptr<factor::Distance> cast(
        const VertexPtr &vp) {
        return (std::dynamic_pointer_cast<factor::Distance>(vp));
      }
      static DistanceFactorPtrVec parse(XmlRpc::XmlRpcValue meas,
                                        TagFactory *tf);
   
    private:
      static DistanceFactorPtr parse(const std::string &name,
                                     XmlRpc::XmlRpcValue meas,
                                     TagFactory *factory);
      double       distance_;
      double       noise_;
      Tag2ConstPtr tag_[2];
      int          corner_[2];
    };
  }
  typedef factor::Distance::DistanceFactorPtr      DistanceFactorPtr;
  typedef factor::Distance::DistanceFactorConstPtr DistanceFactorConstPtr;
  typedef factor::Distance::DistanceFactorPtrVec   DistanceFactorPtrVec;
}
