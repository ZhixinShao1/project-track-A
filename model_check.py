import numpy as np

from src.model.state_space import (
    linearise_continuous,
    controllability_matrix,
    observability_matrix
)


def main():

    # Operating point
    theta_ref = 0.0
    v_ref = 0.06

    # Linearised model
    A, B = linearise_continuous(
        theta_ref,
        v_ref
    )

    print("A matrix:")
    print(A)

    print("\nB matrix:")
    print(B)

    # -----------------------------
    # Controllability
    # -----------------------------

    controllability = controllability_matrix(A, B)

    controllability_rank = np.linalg.matrix_rank(
        controllability
    )

    print("\nControllability matrix:")
    print(controllability)

    print(
        "\nControllability rank:",
        controllability_rank
    )

    # -----------------------------
    # Observability
    # -----------------------------

    # Camera provides full pose:
    # z = [x, y, theta]
    C = np.eye(3)

    observability = observability_matrix(A, C)

    observability_rank = np.linalg.matrix_rank(
        observability
    )

    print("\nObservability matrix:")
    print(observability)

    print(
        "\nObservability rank:",
        observability_rank
    )


if __name__ == "__main__":
    main()