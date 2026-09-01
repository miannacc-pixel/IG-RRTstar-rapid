function P_update = ekf_update_covariance(P_prediction,H,R_measurement)
%EKF_UPDATE_COVARIANCE Joseph-form covariance measurement update.
S=H*P_prediction*H.'+R_measurement; K=(P_prediction*H.')/S; I=eye(size(P_prediction));
P_update=(I-K*H)*P_prediction*(I-K*H).'+K*R_measurement*K.';
P_update=(P_update+P_update.')/2;
end
