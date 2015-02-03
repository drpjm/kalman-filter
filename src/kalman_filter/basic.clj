(ns kalman-filter.basic
  (:require [clojure.core.matrix :as mat]))
(mat/set-current-implementation :vectorz)

; Ok, basic idea(s): use noisy measurements to estimate the "state" of our system.
; The algorithm is recursive. Make a prediction of the future, get current sensor data,
; compare these values, adjust estimate based on the results. (Sounds like basic feedback to me!)

; Notation:
; x - state; u - input; P - system error estimate matrix; Q - process noise; R - measurement noise.
; z - current values to compute the next state.

; This code will follow the Linear Kalman Filter from http://greg.czerniak.info/guides/kalman1/

(defn linear-kalman-filter [A B H Q R]
  ; TODO: Need better pre-conditions.
  #_{:pre [(and (mat/matrix? A) (mat/matrix? B)
               (mat/matrix? H) (mat/matrix? Q) (mat/matrix? R))]}
  "An implementation of the linear Kalman filter for any n-dimensional system. Returns a function
that depends on the current state of a system (x), its covariance matrix (P), given control vector (u),
and sensor measurement (z)."
  (fn [x P u z]
    (let [xpred (mat/add (mat/mmul A x) (mat/mmul B u))
          Ppred (mat/mmul A P (mat/transpose A))
          y (mat/sub z (mat/mmul H xpred))
          S (mat/add (mat/mmul H Ppred (mat/transpose H)) R)
          K (mat/mmul Ppred (mat/transpose H) (mat/inverse S))
          I (mat/identity-matrix (mat/dimension-count A 0))]
      {:state (mat/add xpred (mat/mmul K y))
       :covar-matrix (mat/mmul (mat/sub I (mat/mmul K H)) Ppred)})))

; Invoke the linear kalman filter function to enclose the matrix variables. Only u and z are exposed and a map with
; the current state and updated covariance matrix is returned.