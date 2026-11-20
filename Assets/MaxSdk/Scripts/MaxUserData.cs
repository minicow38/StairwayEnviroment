using System.Collections.Generic;

/// <summary>
/// Contains user identity data to attach to SDK events and ad requests.
///
/// Set plaintext values when available. The native SDK normalizes and hashes PII fields internally before transmission.
/// Email and phone are SHA-256 hashed; pre-hashed normalized SHA-256 hex values are also accepted.
/// User ID is sent as plaintext and must not contain PII.
/// </summary>
public class MaxUserData
{
    /// <summary>
    /// The user's email address. Plaintext is preferred; normalized SHA-256 hex is also accepted.
    /// </summary>
    public string Email { get; set; }

    /// <summary>
    /// The user's phone number. E.164 format is preferred; normalized SHA-256 hex is also accepted.
    /// </summary>
    public string Phone { get; set; }

    /// <summary>
    /// An internal user ID that identifies the user within the publisher's system.
    /// This value is sent as plaintext and must not contain PII.
    /// </summary>
    public string UserId { get; set; }

    internal Dictionary<string, object> ToDictionary()
    {
        var userData = new Dictionary<string, object>();
        if (Email != null) userData["email"] = Email;
        if (Phone != null) userData["phone"] = Phone;
        if (UserId != null) userData["userId"] = UserId;
        return userData;
    }
}
