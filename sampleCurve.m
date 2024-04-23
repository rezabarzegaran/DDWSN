function p = sampleCurve(lambda, curr_step, N_prediction)
% This function is used for sampling Bezier curve.
N_lambda = size(lambda,1);

p = [0 , 0 ,0];
for m=1:N_lambda
    p = p + lambda(m,:)*B(m-1,N_lambda,curr_step,N_prediction);

end

    function c = B(tau, T, k, K_h)
        c = factorial(T)/(factorial(tau)*factorial(T-tau))*(1 - k/K_h)^(T-tau)*(k/K_h)^(tau);
    end
end