(ns kalman-filter.basic-test
  (:require [kalman-filter.basic :as lkf]
            [clojure.core.matrix :as mat]
            [incanter.core :as incanter]
            [incanter.stats :as stats]
            [incanter.charts :as c]))

; To create a linear Kalman filter, you invoke the linear-kalman-filter function, which
; returns a function of four variables: state, covariance, measurement, and input.

; Here is the voltage sensor measurement example. The battery's true voltage is 1.25V. The sensor has 0.25 std dev.

(defn measure-battery [n]
  "Takes n samples from a noisy battery sensor."
  (stats/sample-normal n :mean 1.25 :std 0.25))

(let [A (mat/matrix [1])
      B (mat/matrix [0])
      H (mat/matrix [1])
      Q (mat/matrix [0.0001])
      R (mat/matrix [0.1])
      voltage-update-fn (linear-kalman-filter A B H Q R)]
  ; Store the results for plotting.
  (let [steps 100
        real-battery-vs (repeat steps 1.25)
        estimated-battery-vs (:state (loop [curr-results (voltage-update-fn (mat/matrix [3]) (mat/matrix [1]) 0 (measure-battery 1))
                                           idx 0
                                           state-history []
                                           covar-history []]
                                      (if (>= idx steps)
                                        {:state state-history :covar-matrix covar-history}
                                        (recur (voltage-update-fn (:state curr-results) (:covar-matrix curr-results) 0 (measure-battery 1))
                                               (inc idx)
                                               ; State is returned within a Matrix data structure
                                               (conj state-history (mat/mget (:state curr-results) 0 0))
                                               ; Covariance is returned within a Vector data structure
                                               (conj covar-history (mat/mget (:covar-matrix curr-results) 0))))))
        battery-plot (c/xy-plot (range 0 steps 1) real-battery-vs 
                                :title "Battery Voltage Measurements"
                                :x-label "Steps"
                                :y-label "Voltage (V)")]
    (c/add-lines battery-plot (range 0 steps 1) estimated-battery-vs)
    (incanter/view battery-plot)))
