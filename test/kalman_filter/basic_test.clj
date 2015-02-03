(ns kalman-filter.basic-test
  (:require [kalman-filter.basic :as lkf]
            [clojure.core.matrix :as mat]
            [incanter.core :as incanter]
            [incanter.stats :as stats]
            [incanter.charts :as c]))

; To create a linear Kalman filter, you invoke the linear-kalman-filter function, which
; returns a function of four variables: state, covariance, measurement, and input.

; Here is the voltage sensor measurement example:

(defn measure-battery [n]
  "Takes n samples from a noisy battery sensor."
  (stats/sample-normal n :mean 1.25 :std 0.25))

(let [A (mat/matrix [1])
      B (mat/matrix [0])
      H (mat/matrix [1])
      Q (mat/matrix [0.0001])
      R (mat/matrix [0.1])
      voltage-update-fn (linear-kalman-filter A B H Q R)]
  (voltage-update-fn (mat/matrix [3]) (mat/matrix [1]) 0 (measure-battery 1)))
