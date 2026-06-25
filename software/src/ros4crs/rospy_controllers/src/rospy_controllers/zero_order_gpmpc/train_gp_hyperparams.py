import numpy as np
import torch
import gpytorch


def train_gp(gp_model, likelihood, train_x, train_y):
    gp_model.train()
    likelihood.train()

    max_iter = 100
    optimizer = torch.optim.Adam(
        list(model.parameters()) + list(likelihood.parameters()), lr=0.1
    )
    mll = gpytorch.mlls.ExactMarginalLogLikelihood(likelihood, model)
    for _ in range(max_iter):
        output = model(train_x)
        loss = -mll(output, train_y)
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()


def train_gp_model(
    gp_model, torch_seed=None, training_iterations=200, learning_rate=0.1
):
    if torch_seed is not None:
        torch.manual_seed(torch_seed)

    likelihood = gp_model.likelihood
    train_x = gp_model.train_inputs[0]
    train_y = gp_model.train_targets

    # Find optimal model hyperparameters
    gp_model.train()
    likelihood.train()

    print(f"Training params: {[p for p in gp_model.named_parameters()]}")
    # Use the adam optimizer
    optimizer = torch.optim.Adam(
        filter(lambda p: p.requires_grad, gp_model.parameters()), lr=learning_rate
    )  # Includes GaussianLikelihood parameters

    # "Loss" for GPs - the marginal log likelihood
    # mll = gpytorch.mlls.ExactMarginalLogLikelihood(likelihood, gp_model)
    mll = gpytorch.mlls.VariationalELBO(likelihood, gp_model, num_data=train_y.shape[0])
    print(f"Shape train_y : {train_y.shape}")

    prev_loss = np.inf
    num_loss_below_threshold = 0

    for i in range(training_iterations):
        optimizer.zero_grad()
        output = likelihood(gp_model(train_x))
        loss = torch.sum(-mll(output, train_y))

        if i % 20 == 0:
            print("Iter %d/%d - Loss: %.3f" % (i + 1, training_iterations, loss.item()))

        num_loss_below_threshold = (
            num_loss_below_threshold + 1 if abs(loss - prev_loss) < 5e-4 else 0
        )

        if num_loss_below_threshold > 5:
            print(f"stopping GP optimization early after {i} iterations.")
            break

        prev_loss = loss

        loss.backward()

        optimizer.step()

    gp_model.eval()
    likelihood.eval()
    return gp_model, likelihood
