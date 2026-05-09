import Foundation

public enum ProximityMeshError: LocalizedError {
    case worldSensingNotAuthorized
    case sessionFailure(Error)

    public var errorDescription: String? {
        switch self {
        case .worldSensingNotAuthorized:
            "World sensing authorization was denied. Scene reconstruction requires world sensing permission."
        case .sessionFailure(let error):
            "ARKit session failed: \(error.localizedDescription)"
        }
    }
}
